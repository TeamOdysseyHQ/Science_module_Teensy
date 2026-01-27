// #include <Arduino.h>
// #include <Servo.h>

// Servo myServo;

// const int SERVO_PIN = 41;

// bool toggleState = false;
// int posA = 0;    // 0 degrees
// int posB = 90;   // 90 degrees

// void moveSmooth(int from, int to);

// void setup() {
//   Serial.begin(115200);
//   myServo.attach(SERVO_PIN);

//   myServo.write(posA);  // initial position
//   delay(500);

//   Serial.println("Press 's' to toggle servo 0° <-> 90°");
// }

// void loop() {
//   if (Serial.available()) {
//     char key = Serial.read();

//     if (key == 's' || key == 'S') {
//       toggleState = !toggleState;

//       if (toggleState) {
//         moveSmooth(posA, posB);
//         Serial.println("Servo moved to 90°");
//       } else {
//         moveSmooth(posB, posA);
//         Serial.println("Servo moved to 0°");
//       }
//     }
//   }
// }

// void moveSmooth(int from, int to) {
//   if (from < to) {
//     for (int i = from; i <= to; i++) {
//       myServo.write(i);
//       // delay(10);
//     }
//   } else {
//     for (int i = from; i >= to; i--) {
//       myServo.write(i);
//       // delay(10);
//     }
//   }
// }
