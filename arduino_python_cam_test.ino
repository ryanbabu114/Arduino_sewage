// ============================
// FLOW SENSOR VARIABLES
// ============================
byte statusLed = 13;
byte sensorInterrupt = 0;
byte sensorPin = 2;

float calibrationFactor = 4.5;
volatile byte pulseCount;

float flowRate;
unsigned int flowMilliLitres;
unsigned long totalMilliLitres;
unsigned long oldTime;

// ============================
// ULTRASONIC VARIABLES
// ============================
#define trigPin 6
#define echoPin 7
#define trigPin2 9
#define echoPin2 8

#define SAMPLES 5
#define MAX_DISTANCE 400

#define ALERT_ON_CM   8.0
#define ALERT_OFF_CM  9.5
#define LEVEL_DIFF_THRESHOLD 1.0

// ============================
// BLOCKAGE LOGIC VARIABLES
// ============================
#define FLOW_MIN_THRESHOLD 0.1
#define NO_FLOW_THRESHOLD  0.05
#define BLOCK_CONFIRM_TIME 500

bool alertActive = false;
unsigned long blockageTimer = 0;

// ============================
// PRESSURE SENSOR VARIABLES
// ============================
const int pressurePin = A0;

const float sensorMinVoltage = 0.46;
const float sensorMaxVoltage = 4.5;
const float maxPressure = 1.2;   // MPa

float baseline = 0;
float noiseLevel = 0;

// ============================
// DEVICE INFO
// ============================
const char deviceName[] = "SewerMonitor-01";
const char deviceId[] = "DEV-001";
const char installationDate[] = "2025-12-01";
const char locationName[] = "Choondacherry";
const char latitude[] = "9.726918";
const char longitude[] = "76.726576";

// ============================
// INTERRUPT FUNCTION
// ============================
void pulseCounter() {
  pulseCount++;
}

// ============================
// PRESSURE READING FUNCTION
// ============================
float readPressure() {
  const int samples = 30;
  long sum = 0;

  for (int i = 0; i < samples; i++) {
    sum += analogRead(pressurePin);
    delay(3);
  }

  float adc = sum / (float)samples;

  // INTERNAL reference = 1.1V
  float voltage = adc * (1.1 / 1023.0);

  float pressureMPa = (voltage - sensorMinVoltage) *
                      (maxPressure / (sensorMaxVoltage - sensorMinVoltage));

  if (pressureMPa < 0) pressureMPa = 0;

  return pressureMPa * 1000.0; // kPa
}

// ============================
// STABLE ULTRASONIC FUNCTION
// ============================
float readDistanceStable(int trigPinX, int echoPinX) {
  float total = 0;
  int validReadings = 0;

  for (int i = 0; i < SAMPLES; i++) {
    digitalWrite(trigPinX, LOW);
    delayMicroseconds(5);

    digitalWrite(trigPinX, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPinX, LOW);

    long duration = pulseIn(echoPinX, HIGH, 30000);

    if (duration > 0) {
      float distance = duration * 0.0343 / 2.0;

      if (distance > 2 && distance < MAX_DISTANCE) {
        total += distance;
        validReadings++;
      }
    }

    delay(40);
  }

  if (validReadings == 0) return -1;

  return total / validReadings;
}

// ============================
// SETUP
// ============================
void setup() {
  Serial.begin(9600);

  pinMode(statusLed, OUTPUT);
  digitalWrite(statusLed, HIGH);

  pinMode(sensorPin, INPUT);
  digitalWrite(sensorPin, HIGH);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);

  pinMode(pressurePin, INPUT);

  pulseCount = 0;
  flowRate = 0.0;
  totalMilliLitres = 0;
  oldTime = millis();

  attachInterrupt(sensorInterrupt, pulseCounter, FALLING);

  // Pressure sensor calibration
  analogReference(INTERNAL);

  Serial.println("Calibrating pressure sensor... Keep pipe empty");

  float minP = 1000;
  float maxP = 0;
  float sum = 0;

  for (int i = 0; i < 100; i++) {
    float p = readPressure();
    sum += p;

    if (p < minP) minP = p;
    if (p > maxP) maxP = p;

    delay(20);
  }

  baseline = sum / 100.0;
  noiseLevel = maxP - minP;

  Serial.print("Pressure Baseline: ");
  Serial.print(baseline);
  Serial.println(" kPa");

  Serial.print("Pressure Noise: ");
  Serial.print(noiseLevel);
  Serial.println(" kPa");
}

// ============================
// LOOP
// ============================
void loop() {
  static unsigned long displayTimer = 0;
  static unsigned long flowTimer = 0;

  // ================= FLOW CALCULATION =================
  if ((millis() - flowTimer) > 1000) {
    detachInterrupt(sensorInterrupt);

    flowRate = ((1000.0 / (millis() - oldTime)) * pulseCount) / calibrationFactor;
    oldTime = millis();

    flowMilliLitres = (flowRate / 60.0) * 1000.0;
    totalMilliLitres += flowMilliLitres;

    pulseCount = 0;

    attachInterrupt(sensorInterrupt, pulseCounter, FALLING);

    flowTimer = millis();
  }

  // ================= DISPLAY + BLOCKAGE CHECK =================
  if ((millis() - displayTimer) > 300) {
    float distance = readDistanceStable(trigPin, echoPin);
    delay(70);
    float distance2 = readDistanceStable(trigPin2, echoPin2);
    float pressure = readPressure();

    Serial.println("==================================");
    Serial.println("        DEVICE STATUS");
    Serial.println("==================================");

    Serial.print("Flow Rate     : ");
    Serial.print(flowRate, 2);
    Serial.println(" L/min");

    Serial.print("Total Volume  : ");
    Serial.print(totalMilliLitres);
    Serial.println(" mL");

    Serial.print("Pressure      : ");
    Serial.print(pressure, 2);
    Serial.println(" kPa");

    float pressureThreshold = baseline + (noiseLevel * 2.0);


    Serial.println("----------------------------------");

    if (distance < 0 || distance2 < 0) {
      Serial.println("Ultrasonic    : SENSOR ERROR");
    } else {
      float levelDifference = abs(distance - distance2);
      bool pressureHigh = pressure > pressureThreshold;

      Serial.print("Distance 1    : ");
      Serial.println(distance, 2);

      Serial.print("Distance 2    : ");
      Serial.println(distance2, 2);

      Serial.print("Level Diff    : ");
      Serial.println(levelDifference, 2);

      Serial.print("Pressure High : ");
      Serial.println(pressureHigh ? "YES" : "NO");

      // ================= BLOCKAGE LOGIC =================
      bool blockageDetected =
    distance <= ALERT_ON_CM &&
    levelDifference >= LEVEL_DIFF_THRESHOLD;

      // ================= ALERT TRIGGER =================
      if (!alertActive && blockageDetected) {
        if (blockageTimer == 0)
          blockageTimer = millis();

        if (millis() - blockageTimer >= BLOCK_CONFIRM_TIME) {
          Serial.print("ALERT,");
          Serial.print(locationName); Serial.print(",");
          Serial.print(distance, 2); Serial.print(",");
          Serial.print(distance2, 2); Serial.print(",");
          Serial.print(levelDifference, 2); Serial.print(",");
          Serial.print(flowRate, 2); Serial.print(",");
          Serial.print(pressure, 2); Serial.print(",");
          Serial.print(deviceName); Serial.print(",");
          Serial.print(deviceId); Serial.print(",");
          Serial.print(installationDate); Serial.print(",");
          Serial.print(latitude); Serial.print(",");
          Serial.println(longitude);

          alertActive = true;
          blockageTimer = 0;
        }
      } else if (!blockageDetected) {
        blockageTimer = 0;
      }

      // ================= CLEAR LOGIC =================
       if (alertActive &&
          distance > ALERT_OFF_CM &&
          levelDifference < (LEVEL_DIFF_THRESHOLD * 0.7)) {

        Serial.print("CLEAR,");
        Serial.println(deviceId);

        alertActive = false;
      }
    }

    Serial.println("==================================\n");
    displayTimer = millis();
  }
}
