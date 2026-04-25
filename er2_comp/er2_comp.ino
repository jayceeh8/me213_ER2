#include "CytronMotorDriver.h"
#include <Encoder.h>

const bool DEBUG = false;

const int lf_pwm = 4;
const int lf_dir = 5;
const int lf_e1 = 2;
const int lf_e2 = 3;

const int lb_pwm = 8;
const int lb_dir = 9;
const int lb_e1 = 10;
const int lb_e2 = 11;

const int rf_pwm = 16;
const int rf_dir = 17;
const int rf_e1 = 14;
const int rf_e2 = 15;

const int rb_pwm = 66;
const int rb_dir = 67;
const int rb_e1 = 68;
const int rb_e2 = 69;

const int arm_pwm = 60;
const int arm_dir = 61;
const int arm_e1 = 62;
const int arm_e2 = 63;

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
const int PRINT_INTERVAL = 1000;

float targetDegrees = 120.0;
float kP = 20;
float kI = 80;
float kD = 10;

const int MAX_SPEED = 255;
const int MIN_SPEED = 25;

float prevError = 0;
float integral = 0;
unsigned long lastPIDTime = 0;

bool autoMode = false;

float encoderToDegrees(long counts) {
  return (counts / 288.0) * 360;
}

int runPID(float currentDegrees) {
  unsigned long now = millis();
  float dt = (now - lastPIDTime) / 1000.0;
  lastPIDTime = now;

  float error = targetDegrees - currentDegrees;

  float P = kP * error;

  integral += error * dt;
  integral = constrain(integral, -50, 50);
  float I = kI * integral;

  float derivative = (error - prevError) / dt;
  float D = kD * derivative;
  prevError = error;

  float rawOutput = P + I + D;

  if (rawOutput > 0 && rawOutput < MIN_SPEED)  rawOutput = MIN_SPEED;
  if (rawOutput < 0 && rawOutput > -MIN_SPEED) rawOutput = -MIN_SPEED;

  return constrain((int)rawOutput, -MAX_SPEED, MAX_SPEED);
}

void setDriveSpeed(int left, int right) {
  left_front_motor.setSpeed(left);
  left_back_motor.setSpeed(left);
  right_front_motor.setSpeed(right);
  right_back_motor.setSpeed(right);
}

void stopAll() {
  setDriveSpeed(0, 0);
  arm_motor.setSpeed(0);
}

void serialPrintln(String msg) {
  if(DEBUG) {
    Serial.println(msg);
  }
  Serial1.println(msg);
}

void handleCommand(String input) {
  input.trim();

  if (input == "STOP") {
    autoMode = false;
    integral = 0;
    prevError = 0;
    stopAll();
    serialPrintln("EMERGENCY STOP");
  }
  else if (input == "AUTO") {
    autoMode = true;
    setDriveSpeed(0, 0);
    integral = 0;
    prevError = 0;
    lastPIDTime = millis();
    serialPrintln("AUTO mode started");
  }
  else if (input == "ARM_UP") {
    arm_motor.setSpeed(100);
  }
  else if (input == "ARM_DOWN") {
    arm_motor.setSpeed(-100);
  }
  else if (input == "ARM_STOP") {
    arm_motor.setSpeed(0);
  }
  else if (input == "DRIVE_STOP") {
    setDriveSpeed(0, 0);
  }
  else if (input.startsWith("L") && !autoMode) {
    int commaIndex = input.indexOf(',');
    if (commaIndex == -1) return;
    String leftStr = input.substring(1, commaIndex);
    String rightStr = input.substring(commaIndex + 1);
    leftStr.trim();
    rightStr.trim();
    int leftSpeed = constrain(leftStr.toInt(), -255, 255);
    int rightSpeed = constrain(rightStr.toInt(), -255, 255);
    setDriveSpeed(leftSpeed, rightSpeed);
  }
  else {
    serialPrintln("Unknown command: " + input);
  }
}

void setup() {
  if(DEBUG) {
    Serial.begin(9600);
  }
  Serial1.begin(9600);  // HC-05 on pins 18 (TX1) / 19 (RX1)
  stopAll();

  left_front_encoder.write(0);
  left_back_encoder.write(0);
  right_front_encoder.write(0);
  right_back_encoder.write(0);
  arm_encoder.write(0);
}

void loop() {
  long arm_position      = -arm_encoder.read();
  float arm_degrees      = encoderToDegrees(arm_position);

  long left_front_pos    = -left_front_encoder.read();
  float left_front_degrees = encoderToDegrees(left_front_pos);

  long left_back_pos     = -left_back_encoder.read();
  float left_back_degrees  = encoderToDegrees(left_back_pos);

  long right_front_pos   = -right_front_encoder.read();
  float right_front_degrees = encoderToDegrees(right_front_pos);

  long right_back_pos    = -right_back_encoder.read();
  float right_back_degrees  = encoderToDegrees(right_back_pos);

  // USB serial
  if (DEBUG && Serial.available() > 0) {
    handleCommand(Serial.readStringUntil('\n'));
  }

  // Bluetooth serial
  if (Serial1.available() > 0) {
    handleCommand(Serial1.readStringUntil('\n'));
  }

  if (autoMode) {
    float error = targetDegrees - arm_degrees;
    if (abs(error) < 1.0) {
      arm_motor.setSpeed(0);
      autoMode = false;
      serialPrintln("Target reached");
    } else {
      int pidOutput = runPID(arm_degrees);
      arm_motor.setSpeed(pidOutput);
    }
  }

  unsigned long currentTime = millis();
  if (currentTime - lastPrintTime >= PRINT_INTERVAL) {
    lastPrintTime = currentTime;
    serialPrintln("DEGREES");
    serialPrintln("Arm: "         + String(arm_degrees));
    serialPrintln("Left front: "  + String(left_front_degrees));
    serialPrintln("Left back: "   + String(left_back_degrees));
    serialPrintln("Right front: " + String(right_front_degrees));
    serialPrintln("Right back: "  + String(right_back_degrees));
    serialPrintln("-----------------------------------");
  }
}