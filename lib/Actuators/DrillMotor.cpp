/**
 * @file DCMotor.cpp
 * @brief Simple DC motor control with direction and speed ramping using PWM, tested and works, anticlockwise soil scooping direction
 * @author Rtamanyu N J
 * @date 2024-06-16
 */
#include "DrillMotor.h"
#include "CustomPrint.h"

DrillMotor::DrillMotor(int pwmPin, int dirPin, int slpPin)
    : MOTOR_PWM(pwmPin), MOTOR_DIR(dirPin), MOTOR_SLP(slpPin) {}

void DrillMotor::setup() {
  pinMode(MOTOR_PWM, OUTPUT);
  pinMode(MOTOR_DIR, OUTPUT);
  pinMode(MOTOR_SLP, OUTPUT);

  digitalWrite(MOTOR_SLP, HIGH); // Enable driver
  analogWrite(MOTOR_PWM, 0);
  println("=== Drill Motor Initialized ===");
}

void DrillMotor::handleDirectionChange() {
  if (requestedDir == currentDir) return;

  int previousSpeed = targetSpeed;

  // Ramp down to zero
  rampToSpeed(0);

  // Disable driver to let motor fully stop
  digitalWrite(MOTOR_SLP, LOW);
  delay(dirChangeDelay);

  // Change direction
  if (requestedDir == CLOCKWISE)
    digitalWrite(MOTOR_DIR, HIGH);
  else
    digitalWrite(MOTOR_DIR, LOW);

  currentDir = requestedDir;

  // Re-enable driver and wait for stabilization
  digitalWrite(MOTOR_SLP, HIGH);
  delay(200);

  // Ramp back up
  rampToSpeed(previousSpeed);
}

void DrillMotor::rampToSpeed(int newSpeed) {
  while (currentSpeed != newSpeed) {
    if (currentSpeed < newSpeed)
      currentSpeed++;
    else
      currentSpeed--;

    analogWrite(MOTOR_PWM, currentSpeed);
    delay(rampDelay);
  }
}

void DrillMotor::changeDirection(Direction newDir) {
  requestedDir = newDir;
  handleDirectionChange();
}
void DrillMotor::increaseSpeed() {
  targetSpeed += speedStep;
  if (targetSpeed > 255) targetSpeed = 255;
  rampToSpeed(targetSpeed);
}
void DrillMotor::decreaseSpeed() {
  targetSpeed -= speedStep;
  if (targetSpeed < 0) targetSpeed = 0;
  rampToSpeed(targetSpeed);
}

void DrillMotor::stopMotor() {
  targetSpeed = 0;
  rampToSpeed(targetSpeed);
}