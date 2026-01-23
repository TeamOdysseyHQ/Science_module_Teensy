#include "LinearActuator.h"
#include "CustomPrint.h"

LinearActuator::LinearActuator(int stepPin, int dirPin, int ms1Pin, int ms2Pin, int ms3Pin, int enablePin)
    : STEP_PIN(stepPin), DIR_PIN(dirPin), MS1_PIN(ms1Pin), MS2_PIN(ms2Pin), MS3_PIN(ms3Pin), ENABLE_PIN(enablePin) {}



// ================== SET MICROSTEPPING ==================
void LinearActuator::setMicrostepping(MicrostepMode mode) {
  currentMicrostep = mode;
  switch (mode) {
    case FULL:
      digitalWrite(MS1_PIN, LOW);
      digitalWrite(MS2_PIN, LOW);
      digitalWrite(MS3_PIN, LOW);
      println("LA Microstep: FULL (1/1)");
      break;

    case MICRO_1_4:
      digitalWrite(MS1_PIN, LOW);
      digitalWrite(MS2_PIN, HIGH);
      digitalWrite(MS3_PIN, LOW);
      println("LA Microstep: 1/4");
      break;

    case MICRO_1_8:
      digitalWrite(MS1_PIN, HIGH);
      digitalWrite(MS2_PIN, HIGH);
      digitalWrite(MS3_PIN, LOW);
      println("LA Microstep: 1/8");
      break;

    case MICRO_1_16:
      digitalWrite(MS1_PIN, HIGH);
      digitalWrite(MS2_PIN, HIGH);
      digitalWrite(MS3_PIN, HIGH);
      println("LA Microstep: 1/16");
      break;
  }
}

// ================== ROTATE MOTOR ==================
void LinearActuator::rotateMotor(bool clockwise, int steps) {
  steps *= static_cast<int>(currentMicrostep); // adjust for microstepping
  digitalWrite(DIR_PIN, clockwise ? HIGH : LOW);

  for (int i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(stepDelayMicros);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(stepDelayMicros);
  }
}

// ================== SETUP ==================
void LinearActuator::setup() {
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(MS1_PIN, OUTPUT);
  pinMode(MS2_PIN, OUTPUT);
  pinMode(MS3_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);

  digitalWrite(ENABLE_PIN, LOW); // Enable driver
  setMicrostepping(FULL);
  println("=== Linear Actuator Initialized ===");
}
