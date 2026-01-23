#include "BarrelMotor.h"
#include "CustomPrint.h"
 
BarrelMotor::BarrelMotor(int stepPin, int dirPin, int ms1Pin, int ms2Pin, int ms3Pin, int enablePin)
    : STEP_PIN(stepPin), DIR_PIN(dirPin), MS1_PIN(ms1Pin), MS2_PIN(ms2Pin), MS3_PIN(ms3Pin), ENABLE_PIN(enablePin) {}


void BarrelMotor::setMicrostepping(int mode) {
  switch (mode) {
    case 0: // Full step
      digitalWrite(MS1_PIN, LOW);
      digitalWrite(MS2_PIN, LOW);
      digitalWrite(MS3_PIN, LOW);
      microstepMultiplier = 1;
      println("Barrel Microstep: FULL (1/1)");
      break;

    case 1: // 1/4 step
      digitalWrite(MS1_PIN, LOW);
      digitalWrite(MS2_PIN, HIGH);
      digitalWrite(MS3_PIN, LOW);
      microstepMultiplier = 4;
      println("Barrel Microstep: 1/4");
      break;

    case 2: // 1/8 step
      digitalWrite(MS1_PIN, HIGH);
      digitalWrite(MS2_PIN, HIGH);
      digitalWrite(MS3_PIN, LOW);
      microstepMultiplier = 8;
      println("Barrel Microstep: 1/8");
      break;

    case 3: // 1/16
      digitalWrite(MS1_PIN, HIGH);
      digitalWrite(MS2_PIN, HIGH);
      digitalWrite(MS3_PIN, HIGH);
      microstepMultiplier = 16;
      println("Barrel Microstep: 1/16");
      break;
  }
}

void BarrelMotor::rotateDegrees(float degrees) {
  long steps = (STEPS_PER_REV * microstepMultiplier * degrees) / 360.0;

  digitalWrite(DIR_PIN, HIGH); // change direction if needed

  for (long i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(800);   // speed control
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(800);
  }
}

void BarrelMotor::setup() {
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(MS1_PIN, OUTPUT);
  pinMode(MS2_PIN, OUTPUT);
  pinMode(MS3_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);

  digitalWrite(ENABLE_PIN, LOW); // Enable driver

  setMicrostepping(microstepMode);
  println("=== Barrel Motor Initialized ===");
}
