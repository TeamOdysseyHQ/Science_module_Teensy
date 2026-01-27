// #include <Arduino.h>

// // ================== PIN DEFINITIONS ==================
// #define STEP_PIN 10
// #define DIR_PIN  11
// #define EN_PIN   12

// // ================== MOTOR PARAMETERS =================
// const int stepsPerRev = 200;   // NEMA 17
// int microsteps = 8;            // match TB6600 DIP switches
// int revolutions = 3;           // n revolutions (CHANGE THIS)

// // ================== SPEED CONTROL ====================
// int stepDelay = 1000; // microseconds (lower = faster)

// void rotateMotor(bool anticlockwise);

// // =====================================================
// void setup() {
//   pinMode(STEP_PIN, OUTPUT);
//   pinMode(DIR_PIN, OUTPUT);
//   pinMode(EN_PIN, OUTPUT);

//   digitalWrite(EN_PIN, HIGH); // Enable TB6600 (LOW = enabled)

//   Serial.begin(115200);
//   while (!Serial); // Wait for serial monitor

//   Serial.println("TB6600 Stepper Control Ready");
//   Serial.println("UP Arrow    -> Anticlockwise");
//   Serial.println("DOWN Arrow  -> Clockwise");
// }

// // =====================================================
// void loop() {
// //   if (Serial.available()) {
// //     char c = Serial.read();

// //     // ANSI escape sequence for arrow keys
// //     if (c == 27) {          // ESC
// //       if (Serial.read() == '[') {
// //         char arrow = Serial.read();

// //         if (arrow == 'A') {        // UP arrow
// //           rotateMotor(true);
// //         }
// //         else if (arrow == 'B') {   // DOWN arrow
// //           rotateMotor(false);
// //         }
// //       }
// //     }
// //   }
// digitalWrite(STEP_PIN, HIGH);
//     delayMicroseconds(stepDelay);
//     digitalWrite(STEP_PIN, LOW);
//     delayMicroseconds(stepDelay);
// }

// // =====================================================
// void rotateMotor(bool anticlockwise) {
//   long totalSteps = (long)stepsPerRev * microsteps * revolutions;

//   digitalWrite(DIR_PIN, anticlockwise ? HIGH : LOW);

//   Serial.print("Rotating ");
//   Serial.print(revolutions);
//   Serial.print(" revs ");
//   Serial.println(anticlockwise ? "ANTI-CLOCKWISE" : "CLOCKWISE");

//   for (long i = 0; i < totalSteps; i++) {
//     digitalWrite(STEP_PIN, HIGH);
//     delayMicroseconds(stepDelay);
//     digitalWrite(STEP_PIN, LOW);
//     delayMicroseconds(stepDelay);
//   }

//   Serial.println("Done\n");
// }
