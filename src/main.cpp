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
constexpr int BASE_SPEED = 230;
constexpr int SEARCH_SPEED = 210;

// PID Tuning
constexpr float KP = 0.08;
constexpr float KI = 0.0;
constexpr float KD = 1.8;

constexpr int LINE_CENTER = (SENSOR_COUNT - 1) * 1000 / 2;
constexpr int LINE_PRESENT_SUM_THRESHOLD = 600;
constexpr int LINE_PRESENT_PEAK_THRESHOLD = 300;
constexpr int ERROR_DEADBAND = 300;

// OPTIMIZATION: Allowed to be much higher so the inner wheel can reverse natively in PID
constexpr int MAX_CORRECTION = 500;
constexpr float ERROR_FILTER_ALPHA = 0.8f;

constexpr uint8_t CAL_MAGIC = 0xAB;
constexpr int EEPROM_BASE = 0;
constexpr int CAL_DATA_SIZE = 1 + SENSOR_COUNT * sizeof(uint16_t) * 2;

// --- JUNCTION CONFIGURATION ---
// 0 = Turn Left, 1 = Turn Right, 2 = Go Straight
constexpr int JUNCTION_1_DIR = 1;
constexpr int JUNCTION_2_DIR = 1;
constexpr int JUNCTION_3_DIR = 2;

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
int junctionCount = 0;
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

// OPTIMIZATION: Decouple this from BASE_SPEED.
// 150 is plenty fast for a pivot, but slow enough that sensors won't skip the line.
constexpr int TURN_SPEED = 180;

void executeJunctionTurn(int direction) {
  if (direction == 2) {
    // Go straight: No delays, just immediately return to PID to maintain speed
    return;
  }

  setMotors(-255, -255);
  delay(20); // 30ms is usually enough to stop dead at 250 speed.

  // 2. THE CONTROLLED CREEP: Now that we aren't sliding, move forward slowly to align wheels
  setMotors(150, 150);
  delay(20); // Adjust this slightly up or down to center the wheels perfectly over the intersection

  if (direction == 0) { // Turn Left
    setMotors(-TURN_SPEED, TURN_SPEED);

    // Phase 1: Keep turning until we are safely OFF the current line
    while (true) {
      qtr.readCalibrated(sensorValues);
      if (sensorValues[7] < 100 && sensorValues[6] < 100 && sensorValues[8] < 100) break;
    }

    // Phase 2: ANTICIPATE the new line.
    while (true) {
      qtr.readCalibrated(sensorValues);
      if (sensorValues[4] > JUNCTION_THRESHOLD || sensorValues[5] > JUNCTION_THRESHOLD || sensorValues[6] > JUNCTION_THRESHOLD) break;
    }

    // Phase 3: ACTIVE BRAKING (Counter-steer to kill rotational momentum instantly)
    setMotors(TURN_SPEED, -TURN_SPEED);
    delay(20);
  }
  else if (direction == 1) { // Turn Right
    setMotors(TURN_SPEED, -TURN_SPEED);

    // Phase 1: Keep turning until we are safely OFF the current line
    while (true) {
      qtr.readCalibrated(sensorValues);
      if (sensorValues[7] < 100 && sensorValues[6] < 100 && sensorValues[8] < 100) break;
    }

    // Phase 2: ANTICIPATE the new line.
    while (true) {
      qtr.readCalibrated(sensorValues);
      if (sensorValues[8] > JUNCTION_THRESHOLD || sensorValues[9] > JUNCTION_THRESHOLD || sensorValues[10] > JUNCTION_THRESHOLD) break;
    }

    // Phase 3: ACTIVE BRAKING
    setMotors(-TURN_SPEED, TURN_SPEED);
    delay(20);
  }

  // Reset PID variables so we accelerate cleanly away
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
  qtr.setEmitterPins(41,39);
  delay(1000);
  loadCalibration();

  Serial.println("Starting in 3 seconds...");
  delay(3000);
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
      junctionCount = 0;
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

  // --- JUNCTION DETECTION ---
  bool leftExtreme = (sensorValues[0] > JUNCTION_THRESHOLD) && (sensorValues[1] > JUNCTION_THRESHOLD);
  bool rightExtreme = (sensorValues[SENSOR_COUNT - 1] > JUNCTION_THRESHOLD) && (sensorValues[SENSOR_COUNT - 2] > JUNCTION_THRESHOLD);
  bool centerActive = (sensorValues[7] > JUNCTION_THRESHOLD);

  if ((leftExtreme || rightExtreme) && centerActive && (millis() - lastJunctionTime > JUNCTION_DEBOUNCE_MS)) {
    junctionCount++;
    lastJunctionTime = millis();

    Serial.print("Junction Detected! Count: ");
    Serial.println(junctionCount);

    if (junctionCount % 3 == 1) {
      executeJunctionTurn(JUNCTION_1_DIR);
      return;
    }
    else if (junctionCount % 3 == 2) {
      executeJunctionTurn(JUNCTION_2_DIR);
      return;
    } else if (junctionCount % 3 == 0) {
      executeJunctionTurn(JUNCTION_3_DIR);
      return;
    }
  }

  static uint16_t lastPosition = LINE_CENTER;
  uint16_t position;

  if (isLinePresent && sum > 0) {
    position = weightedSum / sum;
    lastPosition = position;
  } else {
    // OPTIMIZATION: If the line is temporarily lost, this sets a massive error
    // so the PID smoothly spins backward to catch it, keeping forward momentum alive.
    position = lastLineWasRight ? ((SENSOR_COUNT - 1) * 1000) : 0;
  }

  if (millis() - lastPrintTime >= PRINT_INTERVAL_MS) {
    uint16_t rawSensorValues[SENSOR_COUNT];
    qtr.read(rawSensorValues);
    lastPrintTime = millis();
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