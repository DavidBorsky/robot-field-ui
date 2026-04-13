/*
  Arduino Uno bridge sketch for the robot-field-ui project.

  Current responsibilities:
  - receive front/back motor commands over serial
  - clamp and store the command values safely
  - expose a clean place to wire in the real motor driver later
  - leave room for future IR sensor / encoder reporting back to the Pi

  Serial protocol from Python:
    M,<front_output>,<back_output>

  Example:
    M,0.5000,-0.2000
*/

#include <math.h>

struct MotorCommand {
  float front;
  float back;
};

struct EncoderState {
  volatile long count;
  long lastReportedCount;
  float rpm;
  unsigned long lastSampleMs;
};

MotorCommand currentCommand = {0.0f, 0.0f};
EncoderState frontEncoder = {0, 0, 0.0f, 0};
EncoderState backEncoder = {0, 0, 0.0f, 0};

// Replace these once the motor driver wiring is finalized.
const int FRONT_PWM_PIN = -1;
const int FRONT_DIR_PIN = -1;
const int BACK_PWM_PIN = -1;
const int BACK_DIR_PIN = -1;

// Replace these once the encoder wiring is finalized.
const int FRONT_ENCODER_A_PIN = -1;
const int FRONT_ENCODER_B_PIN = -1;
const int BACK_ENCODER_A_PIN = -1;
const int BACK_ENCODER_B_PIN = -1;

const float FRONT_ENCODER_COUNTS_PER_REV = 360.0f;
const float BACK_ENCODER_COUNTS_PER_REV = 360.0f;
const unsigned long SENSOR_REPORT_INTERVAL_MS = 50;

// Future sensor pins can go here once the hardware is wired.
const int LEFT_IR_PIN = -1;
const int RIGHT_IR_PIN = -1;

unsigned long lastSensorReportMs = 0;

float clampUnit(float value) {
  if (value > 1.0f) return 1.0f;
  if (value < -1.0f) return -1.0f;
  return value;
}

int toPwm(float value) {
  value = clampUnit(value);
  return (int)(fabs(value) * 255.0f + 0.5f);
}

bool isForward(float value) {
  return value >= 0.0f;
}

bool encoderPinsConfigured(int aPin, int bPin) {
  return aPin >= 0 && bPin >= 0;
}

void updateEncoderCount(volatile long &countRef, int aPin, int bPin) {
  if (!encoderPinsConfigured(aPin, bPin)) {
    return;
  }

  const int aState = digitalRead(aPin);
  const int bState = digitalRead(bPin);
  countRef += (aState == bState) ? 1 : -1;
}

void onFrontEncoderA() {
  updateEncoderCount(frontEncoder.count, FRONT_ENCODER_A_PIN, FRONT_ENCODER_B_PIN);
}

void onBackEncoderA() {
  updateEncoderCount(backEncoder.count, BACK_ENCODER_A_PIN, BACK_ENCODER_B_PIN);
}

void applySingleMotor(float value, int pwmPin, int dirPin) {
  if (pwmPin < 0 || dirPin < 0) {
    return;
  }

  digitalWrite(dirPin, isForward(value) ? HIGH : LOW);
  analogWrite(pwmPin, toPwm(value));
}

void applyMotorOutputs(const MotorCommand &command) {
  applySingleMotor(command.front, FRONT_PWM_PIN, FRONT_DIR_PIN);
  applySingleMotor(command.back, BACK_PWM_PIN, BACK_DIR_PIN);
}

void stopMotors() {
  currentCommand.front = 0.0f;
  currentCommand.back = 0.0f;
  applyMotorOutputs(currentCommand);
}

bool parseMotorCommand(const String &line, MotorCommand &outCommand) {
  if (!line.startsWith("M,")) {
    return false;
  }

  int firstComma = line.indexOf(',');
  int secondComma = line.indexOf(',', firstComma + 1);
  if (secondComma <= firstComma) {
    return false;
  }

  String frontStr = line.substring(firstComma + 1, secondComma);
  String backStr = line.substring(secondComma + 1);

  outCommand.front = clampUnit(frontStr.toFloat());
  outCommand.back = clampUnit(backStr.toFloat());
  return true;
}

void printCurrentCommand() {
  Serial.print("OK front=");
  Serial.print(currentCommand.front, 4);
  Serial.print(" back=");
  Serial.println(currentCommand.back, 4);
}

float computeRpm(EncoderState &encoder, float countsPerRev) {
  if (countsPerRev <= 0.0f) {
    encoder.lastReportedCount = encoder.count;
    encoder.lastSampleMs = millis();
    encoder.rpm = 0.0f;
    return 0.0f;
  }

  const unsigned long nowMs = millis();
  const unsigned long elapsedMs = nowMs - encoder.lastSampleMs;
  if (elapsedMs == 0) {
    return encoder.rpm;
  }

  const long deltaCounts = encoder.count - encoder.lastReportedCount;
  const float revolutions = deltaCounts / countsPerRev;
  const float elapsedMinutes = elapsedMs / 60000.0f;
  encoder.rpm = revolutions / elapsedMinutes;
  encoder.lastReportedCount = encoder.count;
  encoder.lastSampleMs = nowMs;
  return encoder.rpm;
}

int readEdgePin(int pin) {
  if (pin < 0) {
    return 0;
  }
  return digitalRead(pin) == LOW ? 1 : 0;
}

void printOptionalFloat(float value) {
  if (isnan(value)) {
    return;
  }
  Serial.print(value, 3);
}

void printSensorPacket() {
  computeRpm(frontEncoder, FRONT_ENCODER_COUNTS_PER_REV);
  computeRpm(backEncoder, BACK_ENCODER_COUNTS_PER_REV);

  Serial.print("S,");
  Serial.print(readEdgePin(LEFT_IR_PIN));
  Serial.print(",");
  Serial.print(readEdgePin(RIGHT_IR_PIN));
  Serial.print(",");
  Serial.print("");
  Serial.print(",");
  Serial.print(frontEncoder.count);
  Serial.print(",");
  Serial.print(backEncoder.count);
  Serial.print(",");
  Serial.print(frontEncoder.rpm, 3);
  Serial.print(",");
  Serial.print(backEncoder.rpm, 3);
  Serial.print(",");
  printOptionalFloat(NAN);
  Serial.print(",");
  printOptionalFloat(NAN);
  Serial.print(",");
  printOptionalFloat(NAN);
  Serial.println();
}

void setupMotorPins() {
  if (FRONT_PWM_PIN >= 0) pinMode(FRONT_PWM_PIN, OUTPUT);
  if (FRONT_DIR_PIN >= 0) pinMode(FRONT_DIR_PIN, OUTPUT);
  if (BACK_PWM_PIN >= 0) pinMode(BACK_PWM_PIN, OUTPUT);
  if (BACK_DIR_PIN >= 0) pinMode(BACK_DIR_PIN, OUTPUT);
}

void setupSensorPins() {
  if (LEFT_IR_PIN >= 0) pinMode(LEFT_IR_PIN, INPUT_PULLUP);
  if (RIGHT_IR_PIN >= 0) pinMode(RIGHT_IR_PIN, INPUT_PULLUP);

  if (encoderPinsConfigured(FRONT_ENCODER_A_PIN, FRONT_ENCODER_B_PIN)) {
    pinMode(FRONT_ENCODER_A_PIN, INPUT_PULLUP);
    pinMode(FRONT_ENCODER_B_PIN, INPUT_PULLUP);
    const int interruptNumber = digitalPinToInterrupt(FRONT_ENCODER_A_PIN);
    if (interruptNumber != NOT_AN_INTERRUPT) {
      attachInterrupt(interruptNumber, onFrontEncoderA, CHANGE);
    }
  }

  if (encoderPinsConfigured(BACK_ENCODER_A_PIN, BACK_ENCODER_B_PIN)) {
    pinMode(BACK_ENCODER_A_PIN, INPUT_PULLUP);
    pinMode(BACK_ENCODER_B_PIN, INPUT_PULLUP);
    const int interruptNumber = digitalPinToInterrupt(BACK_ENCODER_A_PIN);
    if (interruptNumber != NOT_AN_INTERRUPT) {
      attachInterrupt(interruptNumber, onBackEncoderA, CHANGE);
    }
  }

  frontEncoder.lastSampleMs = millis();
  backEncoder.lastSampleMs = millis();
}

void setup() {
  Serial.begin(115200);
  setupMotorPins();
  setupSensorPins();
  stopMotors();
  lastSensorReportMs = millis();

  Serial.println("Arduino robot interface ready");
  Serial.println("Expected format: M,<front>,<back>");
}

void loop() {
  const unsigned long nowMs = millis();
  if (nowMs - lastSensorReportMs >= SENSOR_REPORT_INTERVAL_MS) {
    printSensorPacket();
    lastSensorReportMs = nowMs;
  }

  if (Serial.available() <= 0) {
    return;
  }

  String line = Serial.readStringUntil('\n');
  line.trim();

  if (line.length() == 0) {
    return;
  }

  MotorCommand incoming;
  if (parseMotorCommand(line, incoming)) {
    currentCommand = incoming;
    applyMotorOutputs(currentCommand);
    printCurrentCommand();
    return;
  }

  if (line == "STOP") {
    stopMotors();
    printCurrentCommand();
    return;
  }

  Serial.print("ERR unknown command: ");
  Serial.println(line);
}
