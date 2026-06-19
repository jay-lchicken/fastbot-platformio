#include <Arduino.h>
#include <CytronMotorDriver.h>

constexpr uint8_t SENSOR_COUNT = 8;
constexpr uint8_t SENSOR_PINS[SENSOR_COUNT] = {
  A3,A2,A1,A0,A17,A16,A15,A14
};

constexpr uint8_t LEFT_MOTOR_FORWARD_PWM = 2;
constexpr uint8_t LEFT_MOTOR_REVERSE_PWM = 3;
constexpr uint8_t RIGHT_MOTOR_FORWARD_PWM = 1;
constexpr uint8_t RIGHT_MOTOR_REVERSE_PWM = 0;

constexpr bool INVERT_LEFT_MOTOR = false;
constexpr bool INVERT_RIGHT_MOTOR = false;

// Most reflectance sensors read lower on black tape and higher on white floor.
// Set false if your sensor reads higher on the line.
constexpr bool LINE_IS_DARK = true;

constexpr int MOTOR_PWM_MAX = 255;
constexpr int BASE_SPEED = 80;
constexpr int SEARCH_SPEED = 45;

// PID gains. Start conservative; tune Kp first, then Kd, then Ki only if needed.
constexpr float KP = 3;
constexpr float KI = 0.0;
constexpr float KD = 0.0;

constexpr uint32_t CALIBRATION_TIME_MS = 2500;
constexpr int MIN_SENSOR_RANGE = 30;
constexpr int LINE_PRESENT_SUM_THRESHOLD = 900;
constexpr int LINE_PRESENT_PEAK_THRESHOLD = 300;
constexpr int ERROR_DEADBAND = 80;
constexpr int MAX_CORRECTION = 65;
constexpr float ERROR_FILTER_ALPHA = 0.35f;

int sensorMin[SENSOR_COUNT];
int sensorMax[SENSOR_COUNT];
float integral = 0.0f;
int lastError = 0;
bool lastLineWasRight = false;
bool hasFilteredError = false;
float filteredError = 0.0f;

CytronMD leftMotor(PWM_PWM, LEFT_MOTOR_FORWARD_PWM, LEFT_MOTOR_REVERSE_PWM);
CytronMD rightMotor(PWM_PWM, RIGHT_MOTOR_FORWARD_PWM, RIGHT_MOTOR_REVERSE_PWM);

int clampPwm(int value) {
  return constrain(value, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);
}

void setMotor(CytronMD &motor, int speed, bool invert) {
  if (invert) {
    speed = -speed;
  }

  motor.setSpeed(clampPwm(speed));
}

void setDrive(int leftSpeed, int rightSpeed) {
  setMotor(leftMotor, leftSpeed, INVERT_LEFT_MOTOR);
  setMotor(rightMotor, rightSpeed, INVERT_RIGHT_MOTOR);
}

void stopDrive() {
  setDrive(0, 0);
}

void calibrateSensors() {
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    sensorMin[i] = 1023;
    sensorMax[i] = 0;
  }

  const uint32_t start = millis();
  while (millis() - start < CALIBRATION_TIME_MS) {
    for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
      const int value = analogRead(SENSOR_PINS[i]);
      sensorMin[i] = min(sensorMin[i], value);
      sensorMax[i] = max(sensorMax[i], value);
    }
    delay(5);
  }
}

int normalizedLineValue(uint8_t sensorIndex) {
  const int raw = analogRead(SENSOR_PINS[sensorIndex]);
  const int range = max(sensorMax[sensorIndex] - sensorMin[sensorIndex], MIN_SENSOR_RANGE);
  const int normalized = constrain(((raw - sensorMin[sensorIndex]) * 1000L) / range, 0, 1000);
  return LINE_IS_DARK ? 1000 - normalized : normalized;
}

bool readLinePosition(int &error, int &lineStrength) {
  long weightedSum = 0;
  long sum = 0;
  int peak = 0;

  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    const int value = normalizedLineValue(i);

    // SENSOR_PINS is right-to-left. Positive error means the line is to the right.
    const int position = 3500 - (static_cast<int>(i) * 1000);
    weightedSum += static_cast<long>(value) * position;
    sum += value;
    peak = max(peak, value);
  }

  lineStrength = static_cast<int>(sum);
  if (sum < LINE_PRESENT_SUM_THRESHOLD || peak < LINE_PRESENT_PEAK_THRESHOLD) {
    return false;
  }

  error = static_cast<int>(weightedSum / sum);
  lastLineWasRight = error > 0;
  return true;
}

void setup() {
  analogReadResolution(10);

  analogWriteResolution(8);
  analogWriteFrequency(LEFT_MOTOR_FORWARD_PWM, 20000);
  analogWriteFrequency(LEFT_MOTOR_REVERSE_PWM, 20000);
  analogWriteFrequency(RIGHT_MOTOR_FORWARD_PWM, 20000);
  analogWriteFrequency(RIGHT_MOTOR_REVERSE_PWM, 20000);

  stopDrive();
  calibrateSensors();
  Serial.begin(9600);
  delay(2000);
}

void loop() {
  int error = 0;
  int lineStrength = 0;

  if (!readLinePosition(error, lineStrength)) {
    integral = 0.0f;
    lastError = 0;
    hasFilteredError = false;

    const int search = lastLineWasRight ? SEARCH_SPEED : -SEARCH_SPEED;
    setDrive(search, -search);
    delay(10);
    return;
  }

  if (!hasFilteredError) {
    filteredError = error;
    hasFilteredError = true;
  } else {
    filteredError += ERROR_FILTER_ALPHA * (static_cast<float>(error) - filteredError);
  }

  int stableError = static_cast<int>(filteredError + (filteredError >= 0.0f ? 0.5f : -0.5f));
  if (abs(stableError) < ERROR_DEADBAND) {
    stableError = 0;
  }

  integral = constrain(integral + stableError, -12000.0f, 12000.0f);
  const int derivative = stableError - lastError;
  lastError = stableError;

  const int correction = constrain(
      static_cast<int>((KP * stableError) + (KI * integral) + (KD * derivative)),
      -MAX_CORRECTION,
      MAX_CORRECTION);
  const int leftSpeed = BASE_SPEED + correction;
  const int rightSpeed = BASE_SPEED - correction;

  setDrive(leftSpeed, rightSpeed);
  delay(10);
}
