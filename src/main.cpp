#include <Arduino.h>
#include <EEPROM.h>
#include <QTRSensors.h>
#include <CytronMotorDriver.h>

// --- Configuration Constants ---
constexpr uint8_t SENSOR_COUNT = 15;

constexpr uint8_t LEFT_MOTOR_FORWARD_PWM = 0;
constexpr uint8_t LEFT_MOTOR_REVERSE_PWM = 36;
constexpr uint8_t RIGHT_MOTOR_FORWARD_PWM = 1;
constexpr uint8_t RIGHT_MOTOR_REVERSE_PWM = 33;

constexpr int MOTOR_PWM_MAX = 255;
constexpr int BASE_SPEED = 82.5;
constexpr int SEARCH_SPEED = 85;

constexpr float KP = 0.029;
constexpr float KI = 0.00;
constexpr float KD = 0.13;

constexpr int LINE_CENTER = (SENSOR_COUNT - 1) * 1000 / 2;
constexpr int LINE_PRESENT_SUM_THRESHOLD = 45;
constexpr int LINE_PRESENT_PEAK_THRESHOLD = 30;
constexpr int ERROR_DEADBAND = 180;
constexpr int MAX_CORRECTION = 200;
constexpr float ERROR_FILTER_ALPHA = 0.8f;

constexpr uint8_t CAL_MAGIC = 0xAB;
constexpr int EEPROM_BASE = 0;
constexpr int CAL_DATA_SIZE = 1 + SENSOR_COUNT * sizeof(uint16_t) * 2;

// --- Global Objects and Variables ---
QTRSensors qtr;
uint16_t sensorValues[SENSOR_COUNT];

float integral = 0.0f;
int lastError = 0;
bool lastLineWasRight = false;
bool hasFilteredError = false;
float filteredError = 0.0f;

CytronMD leftMotor(PWM_PWM, LEFT_MOTOR_FORWARD_PWM, LEFT_MOTOR_REVERSE_PWM);
CytronMD rightMotor(PWM_PWM, RIGHT_MOTOR_FORWARD_PWM, RIGHT_MOTOR_REVERSE_PWM);

// --- Helper Functions ---

void setMotors(int leftSpeed, int rightSpeed) {
  leftMotor.setSpeed(leftSpeed);
  rightMotor.setSpeed(rightSpeed);
}

void saveCalibration() {
  EEPROM.write(EEPROM_BASE, CAL_MAGIC);

  int address = EEPROM_BASE + 1;
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    EEPROM.put(address, qtr.calibrationOn.minimum[i]);
    address += sizeof(uint16_t);
  }
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    EEPROM.put(address, qtr.calibrationOn.maximum[i]);
    address += sizeof(uint16_t);
  }

  #if defined(ESP8266) || defined(ESP32)
  EEPROM.commit();
  #endif

  Serial.println("Calibration saved to EEPROM.");
}

// NEW: True Auto-Sweep Calibration
void calibrateSensors() {
  Serial.println("Starting AUTO calibration... Please keep hands off.");
  digitalWrite(LED_BUILTIN, HIGH);
  setMotors(0, 0);
  delay(5000);

  // CRITICAL FIX: Erase old calibration memory so bad peaks are forgotten
  qtr.resetCalibration();

  // Define how long the robot swings left and right (adjust if it turns too far/short)
  const int SWEEP_SPEED = 80;
  const int SWEEP_TIME = 1000;

  unsigned long startTime;

  // 1. Sweep Left
  setMotors(-SWEEP_SPEED, SWEEP_SPEED);
  startTime = millis();
  while (millis() - startTime < SWEEP_TIME) { qtr.calibrate(); }

  // 2. Sweep Right (double distance to cross the line entirely)
  setMotors(SWEEP_SPEED, -SWEEP_SPEED);
  startTime = millis();
  while (millis() - startTime < (SWEEP_TIME * 2)) { qtr.calibrate(); }

  // 3. Sweep Left back to roughly the center
  setMotors(-SWEEP_SPEED, SWEEP_SPEED);
  startTime = millis();
  while (millis() - startTime < SWEEP_TIME) { qtr.calibrate(); }

  setMotors(0, 0); // Stop motors
  digitalWrite(LED_BUILTIN, LOW);

  saveCalibration();
  Serial.println("Calibration complete.");
}

void loadCalibration() {
  // Reset and force re-allocation of arrays before loading
  qtr.resetCalibration();
  qtr.calibrate();

  if (EEPROM.read(EEPROM_BASE) == CAL_MAGIC) {
    int address = EEPROM_BASE + 1;
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
      EEPROM.get(address, qtr.calibrationOn.minimum[i]);
      address += sizeof(uint16_t);
    }
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
      EEPROM.get(address, qtr.calibrationOn.maximum[i]);
      address += sizeof(uint16_t);
    }
    Serial.println("Calibration loaded from EEPROM.");
  } else {
    Serial.println("No calibration found in EEPROM. Please calibrate.");
    calibrateSensors();
  }
}

// --- Main Setup and Loop ---

void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);

  #if defined(ESP8266) || defined(ESP32)
  EEPROM.begin(CAL_DATA_SIZE + 10);
  #endif

  // CONFIGURE QTR SENSORS
  qtr.setTypeRC(); // <--- SEE HARDWARE WARNING BELOW
  qtr.setSensorPins((const uint8_t[]){
    14, 15, 16, 17, 18, 19, 20, 21,
    22, 23, 24, 25, 26, 27, 38
  }, SENSOR_COUNT);
  qtr.setEmitterPins(41,39);

  delay(1000);
  loadCalibration();
}

unsigned long lastPrintTime = 0;
constexpr int PRINT_INTERVAL_MS = 250;

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'c' || c == 'C') {
      calibrateSensors();
      integral = 0.0f;
      hasFilteredError = false;
    }
  }

  qtr.readCalibrated(sensorValues);

  long sum = 0;
  long weightedSum = 0;
  uint16_t peak = 0;

  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    uint16_t val = sensorValues[i];
    sum += val;
    weightedSum += (long)val * (i * 1000);
    if (val > peak) peak = val;
  }

  bool isLinePresent = (sum >= LINE_PRESENT_SUM_THRESHOLD) && (peak >= LINE_PRESENT_PEAK_THRESHOLD);

  static uint16_t lastPosition = LINE_CENTER;
  uint16_t position;

  if (isLinePresent && sum > 0) {
    position = weightedSum / sum;
    lastPosition = position;
  } else {
    position = lastLineWasRight ? ((SENSOR_COUNT - 1) * 1000) : 0;
  }

  // DEBUG PRINTING
  if (millis() - lastPrintTime >= PRINT_INTERVAL_MS) {
    uint16_t rawSensorValues[SENSOR_COUNT];
    qtr.read(rawSensorValues);

    Serial.print("Raw: [ ");
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
      Serial.print(rawSensorValues[i]);
      Serial.print(' ');
    }
    Serial.print("] | Cal: [ ");
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
      Serial.print(sensorValues[i]);
      Serial.print(' ');
    }
    Serial.print("] | Pos: ");
    Serial.print(position);
    Serial.print(" | Line: ");
    Serial.println(isLinePresent ? "YES" : "NO");

    lastPrintTime = millis();
  }

  // Line Lost Recovery
  if (!isLinePresent) {
    if (lastLineWasRight) {
      setMotors(SEARCH_SPEED, -SEARCH_SPEED);
    } else {
      setMotors(-SEARCH_SPEED, SEARCH_SPEED);
    }
    return;
  }

  // PID Calculations
  int error = position - LINE_CENTER;

  if (error > 0) lastLineWasRight = true;
  if (error < 0) lastLineWasRight = false;

  if (abs(error) < ERROR_DEADBAND) error = 0;

  if (!hasFilteredError) {
    filteredError = error;
    hasFilteredError = true;
  } else {
    filteredError = (ERROR_FILTER_ALPHA * error) + ((1.0 - ERROR_FILTER_ALPHA) * filteredError);
  }

  integral += filteredError;
  float derivative = filteredError - lastError;
  lastError = filteredError;

  float correction = (KP * filteredError) + (KI * integral) + (KD * derivative);

  if (correction > MAX_CORRECTION) correction = MAX_CORRECTION;
  if (correction < -MAX_CORRECTION) correction = -MAX_CORRECTION;

  int leftSpeed = BASE_SPEED + correction;
  int rightSpeed = BASE_SPEED - correction;

  leftSpeed = constrain(leftSpeed, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);
  rightSpeed = constrain(rightSpeed, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);

  setMotors(leftSpeed, rightSpeed);
}