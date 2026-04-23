#include "CytronMotorDriver.h"
#include <Encoder.h>

// Motor speed max: 255
// const int MOTOR_SPEED = 255/2;

// Motor pin and encoder declarations
// Encoder pins should have interrupt if possible
// See: https://content.arduino.cc/assets/Pinout-Mega2560rev3_latest.pdf
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
  
// Motor and encoder declaration
// PWM pin = 10, DIR pin = 5
CytronMD left_front_motor (PWM_DIR, lf_pwm, lf_dir);
CytronMD left_back_motor (PWM_DIR, lb_pwm, lb_dir);
CytronMD right_front_motor (PWM_DIR, rf_pwm, rf_dir);
CytronMD right_back_motor (PWM_DIR, rb_pwm, rb_dir);
CytronMD arm_motor (PWM_DIR, arm_pwm, arm_dir);

// Both pins should have interrupt capability if possible
Encoder left_front_encoder(lf_e1, lf_e2);
Encoder left_back_encoder(lb_e1, lb_e2);
Encoder right_front_encoder(rf_e1, rf_e2);
Encoder right_back_encoder(rb_e1, rb_e2);
Encoder arm_encoder(arm_e1, arm_e2);

// Serial constants 
unsigned long lastPrintTime = 0;
const int PRINT_INTERVAL = 1000; // print encoder data every 1000ms

// PID constants
float targetDegrees = 120.0;
float kP = 20;
float kI = 80;
float kD = 10;

const int MAX_SPEED = 255;
const int MIN_SPEED = 25;

float prevError = 0;
float integral = 0;
unsigned long lastPIDTime = 0;

// Initial state
bool autoMode = false;

// Helper methods
float encoderToDegrees(long counts) {
  return (counts/288.0) * 360;
}

int runPID(float currentDegrees) {
  unsigned long now = millis();
  float dt = (now - lastPIDTime) / 1000.0;
  lastPIDTime = now;

  float error = targetDegrees - currentDegrees;
  // Serial.println("Error ");
  // Serial.println(error);

  // Proportional
  float P = kP * error;

  // Integral
  integral += error * dt;
  integral = constrain(integral, -50, 50);
  float I = kI * integral;

  // Derivative
  float derivative = (error - prevError) / dt;
  float D = kD * derivative;
  prevError = error;

  float rawOutput = P + I + D;

  if(rawOutput > 0 && rawOutput < MIN_SPEED) {
    rawOutput = MIN_SPEED;
  }

  if(rawOutput < 0 && rawOutput > - MIN_SPEED) {
    rawOutput = -MIN_SPEED;
  }

  int output = constrain((int)rawOutput, -MAX_SPEED, MAX_SPEED);
  return output;
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

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  stopAll();

  // Resets encoder values
  left_front_encoder.write(0);
  left_back_encoder.write(0);
  right_front_encoder.write(0);
  right_back_encoder.write(0);
  arm_encoder.write(0);
}

void loop() {
  // Read encoder values
  long arm_position = -arm_encoder.read();
  float arm_degrees = encoderToDegrees(arm_position);

  long left_front_pos = -left_front_encoder.read();
  float left_front_degrees = encoderToDegrees(left_front_pos);

  long left_back_pos = -left_back_encoder.read();
  float left_back_degrees = encoderToDegrees(left_back_pos);

  long right_front_pos = -right_front_encoder.read();
  float right_front_degrees = encoderToDegrees(right_front_pos);

  long right_back_pos = -right_back_encoder.read();
  float right_back_degrees = encoderToDegrees(right_back_pos);

  if(Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if(input == "STOP") {
      autoMode = false;
      integral = 0;
      prevError = 0;
      stopAll();
      Serial.println("EMERGENCY STOP");
    }
    else if(input == "AUTO") {
      autoMode = true;
      setDriveSpeed(0, 0);
      integral = 0;
      prevError = 0;
      lastPIDTime = millis();
      Serial.println("AUTO mode started");
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
      int leftSpeed = input.substring(1, commaIndex).toInt();
      int rightSpeed = input.substring(commaIndex + 2).toInt();
      
      leftSpeed = constrain(leftSpeed, -255, 255);
      rightSpeed = constrain(rightSpeed, -255, 255);
      setDriveSpeed(leftSpeed, rightSpeed);
    }
    else {
      Serial.print("Unknown command " + input);
    }
  }

  if(autoMode) {
    float error = targetDegrees - arm_degrees;
    if(abs(error) < 1.0) {
      arm_motor.setSpeed(0);
      autoMode = false;
      Serial.println("Target reached");
    }
    else {
      int pidOutput = runPID(arm_degrees);
      arm_motor.setSpeed(pidOutput);
    }
  }

  unsigned long currentTime = millis();
  if (currentTime - lastPrintTime >= PRINT_INTERVAL) {
    lastPrintTime = currentTime;

    Serial.println("DEGREES");
    Serial.println("Arm: " + String(arm_degrees));
    Serial.println("Left front: " + String(left_front_degrees));
    Serial.println("Left back: " + String(left_back_degrees));
    Serial.println("Right front: " + String(right_front_degrees));
    Serial.println("Right back: " + String(right_back_degrees));
    Serial.println("-----------------------------------");
  }
}