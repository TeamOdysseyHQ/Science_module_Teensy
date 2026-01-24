// #include <Arduino.h>
// #include <Servo.h>

// #define SERVO_PIN 9        // PWM-capable pin
// #define CENTER_ANGLE 90    // Neutral position

// Servo myServo;

// bool direction = false;   // false = -90, true = +90

// void setup() {
//   Serial.begin(115200);
//   while (!Serial) ;       // Wait for USB serial

//   myServo.attach(SERVO_PIN);
//   myServo.write(CENTER_ANGLE);  // Move to center

//   Serial.println("Press 's' to toggle servo +90 / -90");
// }

// void loop() {
//   if (Serial.available()) {
//     char key = Serial.read();

//     if (key == 's' || key == 'S') {
//       direction = !direction;

//       int targetAngle;
//       if (direction) {
//         targetAngle = CENTER_ANGLE + 90;   // +90°
//         Serial.println("Servo → +90°");
//       } else {
//         targetAngle = CENTER_ANGLE - 90;   // -90°
//         Serial.println("Servo → -90°");
//       }

//       targetAngle = constrain(targetAngle, 0, 180);
//       myServo.write(targetAngle);
//     }
//   }
// }
