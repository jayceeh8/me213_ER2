#include "CytronMotorDriver.h"
#include <Encoder.h>

const bool DEBUG = true;

// Use hardware Serial2 (pins 16=TX2, 17=RX2) instead of SoftwareSerial
// This is the root cause fix - SoftwareSerial on these pins was dropping incoming bytes

#define lf_pwm 4
#define lf_dir 5
#define lf_e1 2
#define lf_e2 3

#define lb_pwm 8
#define lb_dir 9
#define lb_e1 10
#define lb_e2 11

#define rf_pwm 6
#define rf_dir 7
#define rf_e1 14
#define rf_e2 15

#define rb_pwm 22
#define rb_dir 23
#define rb_e1 68
#define rb_e2 69

#define arm_pwm 24
#define arm_dir 25
#define arm_e1 20
#define arm_e2 21

CytronMD left_front_motor  (PWM_DIR, lf_pwm, lf_dir);
CytronMD left_back_motor   (PWM_DIR, lb_pwm, lb_dir);
CytronMD right_front_motor (PWM_DIR, rf_pwm, rf_dir);
CytronMD right_back_motor  (PWM_DIR, rb_pwm, rb_dir);
CytronMD arm_motor         (PWM_DIR, arm_pwm, arm_dir);

Encoder left_front_encoder(lf_e1, lf_e2);
Encoder left_back_encoder(lb_e1, lb_e2);
Encoder right_front_encoder(rf_e1, rf_e2);
Encoder right_back_encoder(rb_e1, rb_e2);
Encoder arm_encoder(arm_e1, arm_e2);

unsigned long lastPrintTime = 0;
const int PRINT_INTERVAL = 2000;

float targetUp   = 0;
float targetDown = 90.0;

float kP = 2;
float kI = 0.05;
float kD = 2;
const float SLOW_ZONE   = 30.0;  // degrees before target to start slowing
const int APPROACH_FLOOR = 60;   // minimum PWM so arm doesn't stall

const int MAX_SPEED = 200;
const int MIN_SPEED = 0;

float prevError = 0;
float integral  = 0;
unsigned long lastPIDTime = 0;

String autoMode = "";
String btBuffer = "";   // manual buffer for Serial2 reading

float encoderToDegrees(long counts) {
  return (counts / 288.0) * 360;
}

// ── Print helpers ─────────────────────────────────────────────
void serialPrintln(String msg) {
  if (DEBUG) Serial.println(msg);
  Serial2.println(msg);
}

void serialPrint(String msg) {
  if (DEBUG) Serial.print(msg);
  Serial2.print(msg);
}

// ── PID ───────────────────────────────────────────────────────
// int runPID(float targetDegrees, float currentDegrees) {
//   unsigned long now = millis();
//   float dt = max((now - lastPIDTime) / 1000.0, 0.001); // floor at 1ms
//   lastPIDTime = now;

//   float error = targetDegrees - currentDegrees;
//   float P = kP * error;

//   integral += error * dt;
//   integral = constrain(integral, -50, 50);
//   float I = kI * integral;

//   float derivative = (error - prevError) / dt;
//   float D = kD * derivative;
//   prevError = error;

//   float rawOutput = P + I + D;
//   return constrain((int)rawOutput, -MAX_SPEED, MAX_SPEED);
// }

int runPID(float targetDegrees, float currentDegrees) {
  unsigned long now = millis();
  float dt = max((now - lastPIDTime) / 1000.0, 0.001); // floor at 1ms
  lastPIDTime = now;

  float error = targetDegrees - currentDegrees;
  float P = kP * error;

  integral += error * dt;
  integral = constrain(integral, -20, 20); // tightened clamp
  float I = kI * integral;

  float derivative = (error - prevError) / dt;
  float D = kD * derivative;
  prevError = error;

  float rawOutput = P + I + D;

  // Scale max speed down when close to target
  int dynamicMax = MAX_SPEED;
  if (abs(error) < SLOW_ZONE) {
    float fraction = abs(error) / SLOW_ZONE;
    dynamicMax = (int)(APPROACH_FLOOR + fraction * (MAX_SPEED - APPROACH_FLOOR));
  }

  return constrain((int)rawOutput, -dynamicMax, dynamicMax);
}

// ── Arm control ───────────────────────────────────────────────
boolean moveArm(float targetDegrees, float armDegrees) {
  float error = targetDegrees - armDegrees;
  if (abs(error) < 3.0) {
    arm_motor.setSpeed(0);
    autoMode = "";
    serialPrintln("Target reached");
    return false;
  }
  arm_motor.setSpeed(runPID(targetDegrees, armDegrees));
  return true;
}

// ── Drive helpers ─────────────────────────────────────────────
void setDriveSpeed(int left, int right) {
  left_front_motor.setSpeed(left);
  left_back_motor.setSpeed(left);
  right_front_motor.setSpeed(right);
  right_back_motor.setSpeed(-right);
  serialPrintln("motor ON");
}

void stopAll() {
  left_front_motor.setSpeed(0);
  left_back_motor.setSpeed(0);
  right_front_motor.setSpeed(0);
  right_back_motor.setSpeed(0);
  arm_motor.setSpeed(0);
}

// ── Command parser ────────────────────────────────────────────
void handleCommand(String input) {
  input.trim();
  if (input.length() == 0) return;

  // Echo back so Python can confirm receipt
  serialPrintln("GOT:" + input);

  if (input == "STOP") {
    autoMode = "";
    integral = 0;
    prevError = 0;
    stopAll();
    serialPrintln("EMERGENCY STOP");
  }
  else if (input == "AUTO UP") {
    autoMode = input;
    stopAll();
    integral = 0;
    prevError = 0;
    lastPIDTime = millis();
    serialPrintln("AUTO UP started");
  }
  else if (input == "AUTO DOWN") {
    autoMode = input;
    stopAll();
    integral = 0;
    prevError = 0;
    lastPIDTime = millis();
    serialPrintln("AUTO DOWN started");
  }
  else if (input.startsWith("ARM_UP:")) {
  autoMode = "";
  integral = 0;
  prevError = 0;

  int speed = input.substring(7).toInt();  // after "ARM_UP:"
  speed = constrain(speed, 0, 255);

  arm_motor.setSpeed(speed);
  serialPrintln("ARM UP VAR: " + String(speed));
  }
  else if (input.startsWith("ARM_DOWN:")) {
    autoMode = "";
    integral = 0;
    prevError = 0;

    int speed = input.substring(9).toInt();  // after "ARM_DOWN:"
    speed = constrain(speed, 0, 255);

    arm_motor.setSpeed(-speed);  // negative for down
    serialPrintln("ARM DOWN VAR: " + String(speed));
  }
  else if (input == "ARM_STOP") {
    arm_motor.setSpeed(0);
  }
  else if (input == "DRIVE_STOP") {
    setDriveSpeed(0, 0);
  }
  else if (input.startsWith("L")) {
    // Drive command - allowed even in autoMode so STOP works cleanly
    // but skip if arm auto is running
    if (autoMode == "AUTO UP" || autoMode == "AUTO DOWN") {
      serialPrintln("IGNORED: autoMode=" + autoMode);
      return;
    }
    int commaIndex = input.indexOf(',');
    if (commaIndex == -1) { serialPrintln("BAD L CMD"); return; }
    String leftStr  = input.substring(1, commaIndex);
    String rightStr = input.substring(commaIndex + 1);
    leftStr.trim();
    rightStr.trim();
    int leftSpeed  = constrain(leftStr.toInt(),  -255, 255);
    int rightSpeed = constrain(rightStr.toInt(), -255, 255);
    setDriveSpeed(leftSpeed, rightSpeed);
  }
  else {
    serialPrintln("UNKNOWN:" + input);
  }
}

// ── Setup ─────────────────────────────────────────────────────
void setup() {
  if (DEBUG) Serial.begin(9600);

  Serial2.begin(9600);   // hardware UART on pins 16(TX2)/17(RX2)
  delay(500);

  autoMode = "";
  integral  = 0;
  prevError = 0;
  stopAll();

  left_front_encoder.write(0);
  left_back_encoder.write(0);
  right_front_encoder.write(0);
  right_back_encoder.write(0);
  arm_encoder.write(0);

  serialPrintln("READY");
}

// ── Loop ──────────────────────────────────────────────────────
void loop() {
  // Read encoders
  float arm_degrees         = encoderToDegrees(-arm_encoder.read());
  float left_front_degrees  = encoderToDegrees(-left_front_encoder.read());
  float left_back_degrees   = encoderToDegrees(-left_back_encoder.read());
  float right_front_degrees = encoderToDegrees(-right_front_encoder.read());
  float right_back_degrees  = encoderToDegrees(-right_back_encoder.read());

  // USB serial commands (debug)
  if (DEBUG && Serial.available() > 0) {
    handleCommand(Serial.readStringUntil('\n'));
  }

  // BLE/Bluetooth serial - manual byte-by-byte buffer
  // readStringUntil() blocks and misses bytes on SoftwareSerial;
  // manual buffering on hardware serial is reliable
  while (Serial2.available() > 0) {
    char c = (char)Serial2.read();
    if (c == '\n') {
      handleCommand(btBuffer);
      btBuffer = "";
    } else if (c != '\r') {
      btBuffer += c;
      if (btBuffer.length() > 64) btBuffer = ""; // overflow guard
    }
  }

  // Auto arm modes
  if (autoMode == "AUTO UP")   moveArm(targetUp,   arm_degrees);
  if (autoMode == "AUTO DOWN") moveArm(targetDown,  arm_degrees);

  // Periodic encoder print
  unsigned long now = millis();
  if (now - lastPrintTime >= PRINT_INTERVAL) {
    lastPrintTime = now;
    serialPrint("Arm "  + String(arm_degrees));
    serialPrint(", LF:" + String(left_front_degrees));
    serialPrint(", LB:" + String(left_back_degrees));
    serialPrint(", RF:" + String(right_front_degrees));
    serialPrint(", RB:" + String(right_back_degrees));
    serialPrintln("---");
  }
}