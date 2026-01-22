/**
 * @file DCMotor.cpp
 * @brief Simple DC motor control with direction and speed ramping using PWM, tested and works, anticlockwise soil scooping direction
 * @author Rtamanyu N J
 * @date 2024-06-16
 */

#include <Arduino.h>

#define MOTOR_PWM  2
#define MOTOR_DIR  5
#define MOTOR_SLP  3

// when connecting with base center they will give in percentage, must converted to 0-255 for PWM
int targetSpeed = 120;     // Desired speed (0–255)
int currentSpeed = 0;

const int speedStep = 10;
const int rampDelay = 15;     // ms between PWM steps
const int dirChangeDelay = 1000;

enum Direction {
  STOPPED,
  CLOCKWISE,
  ANTICLOCKWISE
};

Direction currentDir = STOPPED;
Direction requestedDir = STOPPED;

void handleDirectionChange();
void rampToSpeed(int newSpeed);


// void setup() {
//   pinMode(MOTOR_PWM, OUTPUT);
//   pinMode(MOTOR_DIR, OUTPUT);
//   pinMode(MOTOR_SLP, OUTPUT);

//   digitalWrite(MOTOR_SLP, HIGH); // Enable driver
//   analogWrite(MOTOR_PWM, 0);

//   Serial.begin(115200);
//   while (!Serial);

//   Serial.println("Single DIR DC Motor Control Ready");
// }

// void loop() {
//   if (Serial.available()) {
//     char c = Serial.read();

//     if (c == 27 && Serial.read() == '[') {
//       char key = Serial.read();

//       switch (key) {
//         case 'C': // RIGHT
//           requestedDir = CLOCKWISE;
//           handleDirectionChange();
//           Serial.println("Clockwise");
//           break;

//         case 'D': // LEFT
//           requestedDir = ANTICLOCKWISE;
//           handleDirectionChange();
//           Serial.println("Anti-clockwise");
//           break;

//         case 'A': // UP
//           targetSpeed += speedStep;
//           if (targetSpeed > 255) targetSpeed = 255;
//           rampToSpeed(targetSpeed);
//           Serial.print("Speed: ");
//           Serial.println(targetSpeed);
//           break;

//         case 'B': // DOWN
//           targetSpeed -= speedStep;
//           if (targetSpeed < 0) targetSpeed = 0;
//           rampToSpeed(targetSpeed);
//           Serial.print("Speed: ");
//           Serial.println(targetSpeed);
//           break;
//       }
//     }
//   }
// }

void handleDirectionChange() {
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

void rampToSpeed(int newSpeed) {
  while (currentSpeed != newSpeed) {
    if (currentSpeed < newSpeed)
      currentSpeed++;
    else
      currentSpeed--;

    analogWrite(MOTOR_PWM, currentSpeed);
    delay(rampDelay);
  }
}
