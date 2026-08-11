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
constexpr int BASE_SPEED = 255;
constexpr int SEARCH_SPEED = 200; // Less aggressive search/recovery speed

// PID Tuning
constexpr float KP = 0.07;
constexpr float KI = 0.001;
constexpr float KD = 1.2;

constexpr int LINE_CENTER = (SENSOR_COUNT - 1) * 1000 / 2;
constexpr int LINE_PRESENT_SUM_THRESHOLD = 600;
constexpr int LINE_PRESENT_PEAK_THRESHOLD = 300;
constexpr int ERROR_DEADBAND = 50;

// OPTIMIZATION: Allowed to be much higher so the inner wheel can reverse natively in PID
constexpr int MAX_CORRECTION = 500;
constexpr float ERROR_FILTER_ALPHA = 0.8f;

constexpr uint8_t CAL_MAGIC = 0xAB;
constexpr int EEPROM_BASE = 0;
constexpr int CAL_DATA_SIZE = 1 + SENSOR_COUNT * sizeof(uint16_t) * 2;

// --- JUNCTION CONFIGURATION ---
// 0 = Turn Left, 1 = Turn Right, 2 = Go Straight
constexpr int JUNCTION_1_DIR = 2;
constexpr int JUNCTION_2_DIR = 1;

constexpr int JUNCTION_THRESHOLD = 600;
constexpr int JUNCTION_DEBOUNCE_MS = 1000;
constexpr int JUNCTION_FORWARD_DELAY = 15;


// --- Global Objects and Variables ---
QTRSensors qtr;
uint16_t sensorValues[SENSOR_COUNT];

float integral = 0.0f;
int lastError = 0;
bool lastLineWasRight = false;
bool hasFilteredError = false;
float filteredError = 0.0f;

// Junction tracking
int totalJunctions = 0;
unsigned long lastJunctionTime = 0;

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

void calibrateSensors() {
  Serial.println("Starting AUTO calibration... Please keep hands off.");
  digitalWrite(LED_BUILTIN, HIGH);
  setMotors(0, 0);

  qtr.resetCalibration();

  const int SWEEP_SPEED = 80;
  const int SWEEP_TIME = 1000;
  unsigned long startTime;

  setMotors(-SWEEP_SPEED, SWEEP_SPEED);
  startTime = millis();
  while (millis() - startTime < SWEEP_TIME) { qtr.calibrate(); }

  setMotors(SWEEP_SPEED, -SWEEP_SPEED);
  startTime = millis();
  while (millis() - startTime < (SWEEP_TIME * 2)) { qtr.calibrate(); }

  setMotors(-SWEEP_SPEED, SWEEP_SPEED);
  startTime = millis();
  while (millis() - startTime < SWEEP_TIME) { qtr.calibrate(); }
  setMotors(-SWEEP_SPEED, SWEEP_SPEED);
  startTime = millis();
  while (millis() - startTime < SWEEP_TIME) { qtr.calibrate(); }

  setMotors(SWEEP_SPEED, -SWEEP_SPEED);
  startTime = millis();
  while (millis() - startTime < (SWEEP_TIME * 2)) { qtr.calibrate(); }

  setMotors(-SWEEP_SPEED, SWEEP_SPEED);
  startTime = millis();
  while (millis() - startTime < SWEEP_TIME) { qtr.calibrate(); }

  setMotors(0, 0);
  digitalWrite(LED_BUILTIN, LOW);

  saveCalibration();
  Serial.println("Calibration complete.");
}

void loadCalibration() {
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

// --- Junction Logic ---

constexpr int TURN_SPEED = 200;

void executeJunctionTurn(int direction) {
  if (direction == 2) {
    // Go straight: No delays, immediately return to PID
    return;
  }

  // Controlled creep to center wheels over the intersection
  setMotors(200, 200);
  delay(30);

  if (direction == 0) { // Turn Left
    setMotors(-TURN_SPEED, TURN_SPEED);

    // Clear the current intersection line
    while (true) {
      qtr.readCalibrated(sensorValues);
      if (sensorValues[7] < 100 && sensorValues[6] < 100 && sensorValues[8] < 100) break;
    }

    // Find the new outgoing line on the left side
    while (true) {
      qtr.readCalibrated(sensorValues);
      if (sensorValues[4] > JUNCTION_THRESHOLD || sensorValues[5] > JUNCTION_THRESHOLD || sensorValues[6] > JUNCTION_THRESHOLD) break;
    }
  }
  else if (direction == 1) { // Turn Right
    setMotors(TURN_SPEED, -TURN_SPEED);

    // Clear the current intersection line
    while (true) {
      qtr.readCalibrated(sensorValues);
      if (sensorValues[7] < 100 && sensorValues[6] < 100 && sensorValues[8] < 100) break;
    }

    // Find the new outgoing line on the right side
    while (true) {
      qtr.readCalibrated(sensorValues);
      if (sensorValues[8] > JUNCTION_THRESHOLD || sensorValues[9] > JUNCTION_THRESHOLD || sensorValues[10] > JUNCTION_THRESHOLD) break;
    }
  }

  // Stop motors briefly to settle before returning to PID
  setMotors(0, 0);
  delay(10);

  // Reset PID variables
  integral = 0.0f;
  hasFilteredError = false;
  lastError = 0;
}

// --- Main Setup and Loop ---

void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);

  #if defined(ESP8266) || defined(ESP32)
  EEPROM.begin(CAL_DATA_SIZE + 10);
  #endif

  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){
    38, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16, 15, 14
  }, SENSOR_COUNT);
  qtr.setEmitterPins(41, 39);
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
      totalJunctions = 0;
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

  // --- JUNCTION & HALF-JUNCTION DETECTION ---
  int activeSensorCount = 0;
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    if (sensorValues[i] > JUNCTION_THRESHOLD) {
      activeSensorCount++;
    }
  }

  bool isJunctionMass = (activeSensorCount >= 6);

  bool leftExtreme = (sensorValues[0] > JUNCTION_THRESHOLD) && (sensorValues[1] > JUNCTION_THRESHOLD);
  bool rightExtreme = (sensorValues[SENSOR_COUNT - 1] > JUNCTION_THRESHOLD) && (sensorValues[SENSOR_COUNT - 2] > JUNCTION_THRESHOLD) ;
  bool centerActive = (sensorValues[7] > JUNCTION_THRESHOLD);

  bool isFullJunction = isJunctionMass && (leftExtreme || rightExtreme) && centerActive;
  bool isHalfJunction = (!isFullJunction) && ((leftExtreme && !rightExtreme) || (rightExtreme && !leftExtreme)) && (activeSensorCount >= 4 && activeSensorCount < 7);

  if ((isFullJunction || isHalfJunction) && (millis() - lastJunctionTime > JUNCTION_DEBOUNCE_MS)) {
    lastJunctionTime = millis();
    totalJunctions++;

    Serial.print("Junction Detected! Index: ");
    Serial.println(totalJunctions);

    int turnDir = 2; // Default to go straight
    if (totalJunctions == 1) {
      turnDir = JUNCTION_1_DIR; // 1st Junction -> Go Straight
    } else if (totalJunctions == 2) {
      turnDir = JUNCTION_2_DIR; // 2nd Junction -> Turn Right
    }

    executeJunctionTurn(turnDir);
    return;
  }

  static uint16_t lastPosition = LINE_CENTER;
  uint16_t position;

  if (isLinePresent && sum > 0) {
    position = weightedSum / sum;
    lastPosition = position;
  } else {
    // --- LINE LOST RECOVERY (Less sharp search speed) ---
    int leftSpin = lastLineWasRight ? SEARCH_SPEED : -SEARCH_SPEED;
    int rightSpin = lastLineWasRight ? -SEARCH_SPEED : SEARCH_SPEED;

    setMotors(leftSpin, rightSpin);

    while (true) {
      qtr.readCalibrated(sensorValues);
      if (sensorValues[6] > LINE_PRESENT_PEAK_THRESHOLD ||
          sensorValues[7] > LINE_PRESENT_PEAK_THRESHOLD ||
          sensorValues[8] > LINE_PRESENT_PEAK_THRESHOLD) {
        break;
      }
    }

    setMotors(-leftSpin, -rightSpin);
    delay(20);

    integral = 0.0f;
    hasFilteredError = false;
    lastError = 0;

    return;
  }

  if (millis() - lastPrintTime >= PRINT_INTERVAL_MS) {
    uint16_t rawSensorValues[SENSOR_COUNT];
    qtr.read(rawSensorValues);
    lastPrintTime = millis();
  }

  // --- PID Calculations ---
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

  // --- DYNAMIC KP MULTIPLIER ---
  int absError = abs(error);
  float kpm = 1.0f;

  if (absError > 600) {
    long kpm_mapped = map(absError, 600, 7000, 12, 40);
    kpm_mapped = constrain(kpm_mapped, 12, 40);
    kpm = kpm_mapped / 10.0f;
  }

  float currentKP = KP * kpm;
  float correction = (currentKP * filteredError) + (KI * integral) + (KD * derivative);

  if (correction > 1000) correction = 1000;
  if (correction < -1000) correction = -1000;

  int leftSpeed = BASE_SPEED + correction;
  int rightSpeed = BASE_SPEED - correction;

  leftSpeed = constrain(leftSpeed, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);
  rightSpeed = constrain(rightSpeed, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);

  setMotors(leftSpeed, rightSpeed);
}