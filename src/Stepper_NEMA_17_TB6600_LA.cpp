// #include <AccelStepper.h>

// /* ================= TB6600 PIN DEFINITIONS ================= */
// // important - Motor driver - TB6600 pins
// #define STEP_PIN   7  // PUL+
// #define DIR_PIN    8   // DIR+
// #define ENABLE_PIN 9   // ENA+

// // important - Motor Driver - b126
// // #define STEP_PIN   10  // PUL+
// // #define DIR_PIN    11   // DIR+
// // #define ENABLE_PIN 12   // ENA+, this must be enabled high

// /* ================= STEPPER ================= */
// AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// /* ================= USER SETTINGS ================= */

// // 🔁 Revolutions per command
// float n_revolutions = 2.0;

// // 🚀 Speed & acceleration
// float motorSpeed = 2500.0;   // steps/sec
// float motorAccel = 4000.0;

// // ⚙️ NEMA 17 motor
// const int fullStepsPerRev = 200;

// /*
// TB6600 MICROSTEP MULTIPLIER (MATCH DIP SWITCH)

// Full step  -> 1
// 1/2 step   -> 2
// 1/4 step   -> 4
// 1/8 step   -> 8
// 1/16 step  -> 16
// 1/32 step  -> 32
// */
// const int microstepMultiplier = 16;   // <<< SET THIS ONLY

// /* ================= STATE ================= */
// bool motorRunning = false;

// /* ================= MOVE MOTOR ================= */
// void moveMotor(bool anticlockwise)
// {
//   long steps = n_revolutions *
//                fullStepsPerRev *
//                microstepMultiplier;

//   if (!anticlockwise)
//     steps = -steps;

//   stepper.move(steps);
//   motorRunning = true;

//   Serial.println(anticlockwise ?
//     "Rotating ANTICLOCKWISE" :
//     "Rotating CLOCKWISE");
// }

// /* ================= SETUP ================= */
// void setup()
// {
//   Serial.begin(115200);

//   pinMode(ENABLE_PIN, OUTPUT);
//   digitalWrite(ENABLE_PIN, LOW);   // Enable TB6600
// //   digitalWrite(ENABLE_PIN, HIGH);   // Enable B126

//   stepper.setMaxSpeed(motorSpeed);
//   stepper.setAcceleration(motorAccel);

//   Serial.println("TB6600 Stepper Ready");
//   Serial.print("Microstep multiplier: ");
//   Serial.println(microstepMultiplier);
//   Serial.println("UP    : Anticlockwise");
//   Serial.println("DOWN  : Clockwise");
// }

// /* ================= LOOP ================= */
// void loop()
// {
//   stepper.run();

//   // Detect motion completion
//   if (motorRunning && stepper.distanceToGo() == 0)
//   {
//     motorRunning = false;
//     Serial.println("Motion Complete");
//   }

//   // Block arrow keys while motor is moving
//   if (motorRunning)
//     return;

//   if (Serial.available())
//   {
//     char c = Serial.read();

//     // Arrow keys (ESC [ A / B)
//     if (c == 27 && Serial.available() >= 2)
//     {
//       Serial.read();        // '['
//       char arrow = Serial.read();

//       if (arrow == 'A')     // UP arrow
//         moveMotor(true);
//       else if (arrow == 'B')// DOWN arrow
//         moveMotor(false);
//     }
//   }
// }
