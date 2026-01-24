// #include <Arduino.h>
#include "PHServo.h"
#include "CustomPrint.h"

PHServo::PHServo(int servoPin) : SERVO_PIN(servoPin) {}

void PHServo::setup() {
 
  myServo.attach(SERVO_PIN);
  myServo.write(CENTER_ANGLE);  // Move to center

  println("Servo setup complete. Ready to toggle position.");
}

void PHServo::togglePosition() {
  direction = !direction;

  int targetAngle;
  if (direction) {
    targetAngle = CENTER_ANGLE + 90;   // +90°
    println("Servo → +90°");
  } else {
    targetAngle = CENTER_ANGLE - 90;   // -90°
    println("Servo → -90°");
  }

  targetAngle = constrain(targetAngle, 0, 180);
  myServo.write(targetAngle);
}
