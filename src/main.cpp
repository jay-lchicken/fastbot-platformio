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
constexpr int SEARCH_SPEED = 235;

// PID Tuning
constexpr float KP = 0.12;
constexpr float KI = 0.001;
constexpr float KD = 1;

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
constexpr int JUNCTION_1_DIR = 0;
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
constexpr int TURN_SPEED = 220;

void executeJunctionTurn(int direction) {
  if (direction == 2) {
    // Go straight: No delays, just immediately return to PID to maintain speed
    return;
  }

  setMotors(-255, -255);
  delay(20); // 30ms is usually enough to stop dead at 250 speed.

  // 2. THE CONTROLLED CREEP: Now that we aren't sliding, move forward slowly to align wheels
  setMotors(210, 210);
  delay(15); // Adjust this slightly up or down to center the wheels perfectly over the intersection

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

  // Serial.println("Starting in 3 seconds...");
  // delay(3000);

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

  // 1. Count exactly how many sensors are currently seeing black
  int activeSensorCount = 0;
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    if (sensorValues[i] > JUNCTION_THRESHOLD) {
      activeSensorCount++;
    }
  }

  // 2. A normal curve covers ~4-6 sensors. A junction covers a wide horizontal band.
  // Adjust this threshold (e.g., 7, 8, or 9) depending on your line thickness.
  bool isJunctionMass = (activeSensorCount >= 8);

  // To avoid false positives on corners, a junction requires the extremes AND the center to see the line
  bool leftExtreme = (sensorValues[0] > JUNCTION_THRESHOLD) && (sensorValues[1] > JUNCTION_THRESHOLD) && (sensorValues[2] > JUNCTION_THRESHOLD);
  bool rightExtreme = (sensorValues[SENSOR_COUNT - 1] > JUNCTION_THRESHOLD) && (sensorValues[SENSOR_COUNT - 2] > JUNCTION_THRESHOLD);
  bool centerActive = (sensorValues[7] > JUNCTION_THRESHOLD);

  // 3. Require the mass check alongside your position checks
  if (isJunctionMass && (leftExtreme || rightExtreme) && centerActive && (millis() - lastJunctionTime > JUNCTION_DEBOUNCE_MS)) {
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
    // --- LINE LOST RECOVERY ---
    // Spin in the direction we last saw the line
    int leftSpin = lastLineWasRight ? SEARCH_SPEED : -SEARCH_SPEED;
    int rightSpin = lastLineWasRight ? -SEARCH_SPEED : SEARCH_SPEED;

    setMotors(leftSpin, rightSpin);

    // Keep spinning in a tight loop UNTIL the middle sensors see the line
    while (true) {
      qtr.readCalibrated(sensorValues);

      // Check sensors 6, 7 (dead center), and 8
      if (sensorValues[6] > LINE_PRESENT_PEAK_THRESHOLD ||
          sensorValues[7] > LINE_PRESENT_PEAK_THRESHOLD ||
          sensorValues[8] > LINE_PRESENT_PEAK_THRESHOLD) {
        break; // Line found by the middle sensors!
          }
    }

    // ACTIVE BRAKING: Briefly counter-steer to kill the spinning momentum
    // so it doesn't overshoot the line it just found.
    setMotors(-leftSpin, -rightSpin);
    delay(20);

    // Reset PID variables so it accelerates cleanly forward without jerking
    integral = 0.0f;
    hasFilteredError = false;
    lastError = 0;

    return; // Restart the loop immediately with the line now centered
  }

  if (millis() - lastPrintTime >= PRINT_INTERVAL_MS) {
    uint16_t rawSensorValues[SENSOR_COUNT];
    qtr.read(rawSensorValues);
    lastPrintTime = millis();
  }

  // PID Calculations
  // --- PID Calculations ---
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
  float kpm = 1.0f; // Default multiplier for straight lines

  if (absError > 600) { // Changed from 1500: Reacts to mild curves much sooner
    // Start gently increasing the multiplier, maxing out at 4.0x for extreme hairpins
    long kpm_mapped = map(absError, 600, 7000, 12, 40);
    kpm_mapped = constrain(kpm_mapped, 12, 40); // 1.2x at mild curves, up to 4.0x
    kpm = kpm_mapped / 10.0f;
  }

  float currentKP = KP * kpm;
  float correction = (currentKP * filteredError) + (KI * integral) + (KD * derivative);

  // Boost max correction buffer so the math is allowed to hit full 255/-255 output
  if (correction > 1000) correction = 1000;
  if (correction < -1000) correction = -1000;

  int leftSpeed = BASE_SPEED + correction;
  int rightSpeed = BASE_SPEED - correction;

  leftSpeed = constrain(leftSpeed, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);
  rightSpeed = constrain(rightSpeed, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);

  setMotors(leftSpeed, rightSpeed);
}

// unsigned long lastPrintTime = 0;
// constexpr int PRINT_INTERVAL_MS = 250;
//
// void loop() {
//   if (Serial.available()) {
//     char c = Serial.read();
//     if (c == 'c' || c == 'C') {
//       calibrateSensors();
//       integral = 0.0f;
//       hasFilteredError = false;
//       junctionCount = 0; // Reset junction count on calibration
//     }
//   }
//
//   // Read calibrated values (0 to 1000) into the array
//   qtr.readCalibrated(sensorValues);
//
//   long sum = 0;
//   long weightedSum = 0;
//   uint16_t peak = 0;
//
//   for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
//     uint16_t val = sensorValues[i];
//     sum += val;
//     weightedSum += (long)val * (i * 1000);
//     if (val > peak) peak = val;
//   }
//
//   bool isLinePresent = (sum >= LINE_PRESENT_SUM_THRESHOLD) && (peak >= LINE_PRESENT_PEAK_THRESHOLD);
//
//   // --- JUNCTION DETECTION ---
//   // To avoid false positives on corners, a junction requires the extremes AND the center to see the line
//   bool leftExtreme = (sensorValues[0] > JUNCTION_THRESHOLD) && (sensorValues[1] > JUNCTION_THRESHOLD);
//   bool rightExtreme = (sensorValues[SENSOR_COUNT - 1] > JUNCTION_THRESHOLD) && (sensorValues[SENSOR_COUNT - 2] > JUNCTION_THRESHOLD);
//   bool centerActive = (sensorValues[7] > JUNCTION_THRESHOLD);
//
//   if ((leftExtreme || rightExtreme) && centerActive && (millis() - lastJunctionTime > JUNCTION_DEBOUNCE_MS)) {
//     junctionCount++;
//     lastJunctionTime = millis();
//
//     Serial.print("Junction Detected! Count: ");
//     Serial.println(junctionCount);
//
//     if (junctionCount % 3 == 1) {
//       executeJunctionTurn(JUNCTION_1_DIR);
//       return; // Skip the rest of the loop for this iteration
//     }
//     else if (junctionCount % 3 == 2) {
//       executeJunctionTurn(JUNCTION_2_DIR);
//       return; // Skip the rest of the loop for this iteration
//     }else if (junctionCount % 3 == 0) {
//       executeJunctionTurn(JUNCTION_3_DIR);
//
//     }
//   }
//
//   static uint16_t lastPosition = LINE_CENTER;
//   uint16_t position;
//
//   if (isLinePresent && sum > 0) {
//     position = weightedSum / sum;
//     lastPosition = position;
//   } else {
//     position = lastLineWasRight ? ((SENSOR_COUNT - 1) * 1000) : 0;
//   }
//
//   // --- PRINT CALIBRATED VALUES ---
//   if (millis() - lastPrintTime >= PRINT_INTERVAL_MS) {
//     for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
//       Serial.print(sensorValues[i]);
//       Serial.print('\t'); // Use a tab character to space out the columns neatly
//     }
//     Serial.println(); // Print a new line at the end of the array
//     lastPrintTime = millis();
//   }
//
//   // Line Lost Recovery
//   if (!isLinePresent) {
//     if (lastLineWasRight) {
//       setMotors(SEARCH_SPEED, -SEARCH_SPEED);
//     } else {
//       setMotors(-SEARCH_SPEED, SEARCH_SPEED);
//     }
//     return;
//   }
//
//   // PID Calculations
//   int error = position - LINE_CENTER;
//
//   if (error > 0) lastLineWasRight = true;
//   if (error < 0) lastLineWasRight = false;
//
//   if (abs(error) < ERROR_DEADBAND) error = 0;
//
//   if (!hasFilteredError) {
//     filteredError = error;
//     hasFilteredError = true;
//   } else {
//     filteredError = (ERROR_FILTER_ALPHA * error) + ((1.0 - ERROR_FILTER_ALPHA) * filteredError);
//   }
//
//   integral += filteredError;
//   float derivative = filteredError - lastError;
//   lastError = filteredError;
//
//   float correction = (KP * filteredError) + (KI * integral) + (KD * derivative);
//
//   if (correction > MAX_CORRECTION) correction = MAX_CORRECTION;
//   if (correction < -MAX_CORRECTION) correction = -MAX_CORRECTION;
//
//   int leftSpeed = BASE_SPEED + correction;
//   int rightSpeed = BASE_SPEED - correction;
//
//   leftSpeed = constrain(leftSpeed, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);
//   rightSpeed = constrain(rightSpeed, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);
//
//   setMotors(leftSpeed, rightSpeed);
// }