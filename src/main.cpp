#include <Arduino.h>
#include <EEPROM.h>
#include <QTRSensors.h>
#include <CytronMotorDriver.h>

// --- Configuration Constants ---
constexpr uint8_t SENSOR_COUNT = 16;

constexpr uint8_t LEFT_MOTOR_FORWARD_PWM = 36;
constexpr uint8_t LEFT_MOTOR_REVERSE_PWM = 35;
constexpr uint8_t RIGHT_MOTOR_FORWARD_PWM = 33;
constexpr uint8_t RIGHT_MOTOR_REVERSE_PWM = 34;


constexpr int MOTOR_PWM_MAX = 255;
constexpr int BASE_SPEED = 200;
constexpr int SEARCH_SPEED = 150;

constexpr float KP = 0.03;
constexpr float KI = 0.000;
constexpr float KD = 20;

constexpr int LINE_CENTER = (SENSOR_COUNT - 1) * 1000 / 2;
constexpr int LINE_PRESENT_SUM_THRESHOLD = 45;
constexpr int LINE_PRESENT_PEAK_THRESHOLD = 30;
constexpr int ERROR_DEADBAND = 80;
constexpr int MAX_CORRECTION = 100;
constexpr float ERROR_FILTER_ALPHA = 0.35f;

constexpr uint8_t CAL_MAGIC = 0xAB;
constexpr int EEPROM_BASE = 0;
constexpr int CAL_DATA_SIZE = 1 + SENSOR_COUNT * sizeof(uint16_t) * 2;

// --- Global Objects and Variables ---
QTRSensors qtr;
uint16_t sensorValues[SENSOR_COUNT];
uint16_t savedMin[SENSOR_COUNT];
uint16_t savedMax[SENSOR_COUNT];

float integral = 0.0f;
int lastError = 0;
bool lastLineWasRight = false;
bool hasFilteredError = false;
float filteredError = 0.0f;

CytronMD leftMotor(PWM_DIR, LEFT_MOTOR_FORWARD_PWM, LEFT_MOTOR_REVERSE_PWM);
CytronMD rightMotor(PWM_DIR, RIGHT_MOTOR_FORWARD_PWM, RIGHT_MOTOR_REVERSE_PWM);

// --- Helper Functions ---

// Safely sets motor speeds considering inversion constants
void setMotors(int leftSpeed, int rightSpeed) {


  leftMotor.setSpeed(leftSpeed);
  rightMotor.setSpeed(rightSpeed);
}

// Write calibration arrays to EEPROM
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

  // Necessary for ESP32/ESP8266; safely ignored on AVR boards
  #if defined(ESP8266) || defined(ESP32)
  EEPROM.commit();
  #endif

  Serial.println("Calibration saved to EEPROM.");
}

// Standard calibration loop
void calibrateSensors() {
  setMotors(0, 0); // Ensure motors are stopped
  Serial.println("Starting calibration... move the robot over the line.");
  digitalWrite(LED_BUILTIN, HIGH);

  // 400 loops takes ~10 seconds
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }

  digitalWrite(LED_BUILTIN, LOW);
  saveCalibration();
  Serial.println("Calibration complete.");
}

// Load calibration from EEPROM if it exists
void loadCalibration() {
  // Call calibrate once to force the QTR library to allocate its internal memory arrays
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

  // Initialize EEPROM size if running on ESP32/ESP8266
  #if defined(ESP8266) || defined(ESP32)
  EEPROM.begin(CAL_DATA_SIZE + 10);
  #endif

  // Configure QTR Sensors
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){
    A0, A1, A2, A3, A4, A5, A6, A7,
    A8, A9, A10, A11, A12, A13, A14, A15
  }, SENSOR_COUNT);
  qtr.setEmitterPin(12);

  delay(1000);

  // Attempt to load calibration from memory
  loadCalibration();

}

// Add this global variable right above your loop() function
unsigned long lastPrintTime = 0;
constexpr int PRINT_INTERVAL_MS = 250; // Print 4 times a second

void loop() {
  // 1. Check for manual calibration trigger
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'c' || c == 'C') {
      calibrateSensors();
      integral = 0.0f; // Reset PID memory after calibration
      hasFilteredError = false;
    }
  }
  // 1. Read values manually (bypassing readLineBlack's 50 threshold)
  qtr.readCalibrated(sensorValues);

  // 2. Calculate our own sum, peak, and weighted position
  long sum = 0;
  long weightedSum = 0;
  uint16_t peak = 0;

  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    uint16_t val = sensorValues[i];

    // Optional: subtract a tiny bit of noise floor to make it cleaner,
    // but we'll leave it out since your values are so low.
    sum += val;
    weightedSum += (long)val * (i * 1000);

    if (val > peak) {
      peak = val;
    }
  }

  bool isLinePresent = (sum >= LINE_PRESENT_SUM_THRESHOLD) && (peak >= LINE_PRESENT_PEAK_THRESHOLD);

  // 3. Calculate position using center-of-mass math
  static uint16_t lastPosition = LINE_CENTER;
  uint16_t position;

  if (isLinePresent && sum > 0) {
    position = weightedSum / sum;
    lastPosition = position;
  } else {
    // If the line is lost, push the position to the extreme edge it was last seen on
    position = lastLineWasRight ? ((SENSOR_COUNT - 1) * 1000) : 0;
  }
  // --- DEBUG PRINTING (Non-blocking) ---
  if (millis() - lastPrintTime >= PRINT_INTERVAL_MS) {
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
      Serial.print(sensorValues[i]);
      Serial.print('\t');
    }
    Serial.print("| Pos: ");
    Serial.print(position);
    Serial.print(" | Line: ");
    Serial.println(isLinePresent ? "YES" : "NO");

    lastPrintTime = millis();
  }
  // -------------------------------------

  // 4. Line Lost Recovery Routine
  if (!isLinePresent) {
    if (lastLineWasRight) {
      setMotors(SEARCH_SPEED, -SEARCH_SPEED);
    } else {
      setMotors(-SEARCH_SPEED, SEARCH_SPEED);
    }
    return; // Skip PID while searching
  }

  // 5. PID Calculations
  int error = position - LINE_CENTER;

  // Remember direction for line loss recovery
  if (error > 0) lastLineWasRight = true;
  if (error < 0) lastLineWasRight = false;

  // Apply deadband
  if (abs(error) < ERROR_DEADBAND) {
    error = 0;
  }

  // Low-pass filter the error
  if (!hasFilteredError) {
    filteredError = error;
    hasFilteredError = true;
  } else {
    filteredError = (ERROR_FILTER_ALPHA * error) + ((1.0 - ERROR_FILTER_ALPHA) * filteredError);
  }

  // Calculate Terms
  integral += filteredError;
  float derivative = filteredError - lastError;
  lastError = filteredError;

  float correction = (KP * filteredError) + (KI * integral) + (KD * derivative);

  // Constrain correction
  if (correction > MAX_CORRECTION) correction = MAX_CORRECTION;
  if (correction < -MAX_CORRECTION) correction = -MAX_CORRECTION;

  // 6. Apply to Motors
  int leftSpeed = BASE_SPEED + correction;
  int rightSpeed = BASE_SPEED - correction;

  // Constrain final speeds to valid PWM range
  leftSpeed = constrain(leftSpeed, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);
  rightSpeed = constrain(rightSpeed, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);

  setMotors(leftSpeed, rightSpeed);
}