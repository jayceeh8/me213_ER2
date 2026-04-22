#include "CytronMotorDriver.h"
#include <Encoder.h>

// Motor speed max: 255
const int MOTOR_SPEED = 255/2;
  
// Motor and encoder declaration
// PWM pin = 10, DIR pin = 5
CytronMD motor (PWM_DIR, 10, 5);
// Both pins should have interrupt capability if possible
Encoder encoder(2, 3);

// Serial constants 
unsigned long lastPrintTime = 0;
const int PRINT_INTERVAL = 100; // print encoder data every 100ms

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

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  motor.setSpeed(0);
  encoder.write(0);
}

void loop() {
  long position = -encoder.read();
  float degrees = encoderToDegrees(position);

  if(Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');

    if(input == "STOP") {
      autoMode = false;
      integral = 0;
      prevError = 0;
      motor.setSpeed(0);
      Serial.println("EMERGENCY STOP");
    }
    else if(input == "AUTO") {
      autoMode = true;
      integral = 0;
      prevError = 0;
      lastPIDTime = millis();
      Serial.println("AUTO mode started");
    }
    else if (input.startsWith("V")) {
      int speed = input.substring(1).toInt();
      speed = constrain(speed, -255, 255);
      motor.setSpeed(speed);
    }
  }

  if(autoMode) {
    float error = targetDegrees - degrees;
    if(abs(error) < 1.0) {
      motor.setSpeed(0);
      autoMode = false;
      Serial.println("Target reached");
    }
    else {
      int pidOutput = runPID(degrees);
      motor.setSpeed(pidOutput);
    }
  }

  unsigned long currentTime = millis();
  if (currentTime - lastPrintTime >= PRINT_INTERVAL) {
    lastPrintTime = currentTime;

    Serial.print("Count: ");
    Serial.print(position);
    Serial.print("  Degrees: ");
    Serial.println(degrees);
  }
}