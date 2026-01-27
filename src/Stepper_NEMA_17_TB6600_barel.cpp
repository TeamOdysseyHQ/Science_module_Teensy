// #include <AccelStepper.h>

// // ================= PIN DEFINITIONS =================
// #define STEP_PIN    7
// #define DIR_PIN     8
// #define ENABLE_PIN  9

// // ================= MOTOR SETUP =====================
// AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// // ================= MOTOR SPECS =====================
// const int BASE_STEPS_PER_REV = 200;   // NEMA 17 (1.8°)
// const int MICROSTEP = 16;             // MUST match TB6600 DIP switches
// float ROTATE_DEG = 240.0;

// // ================= CALCULATED ======================
// int STEPS_PER_REV = BASE_STEPS_PER_REV * MICROSTEP;
// int STEPS_60_DEG  = (STEPS_PER_REV * ROTATE_DEG) / 360.0;

// // ================= SPEED CONTROL ===================
// float stepperSpeed = 2500.0;           
// float stepperAccel = 2400.0;

// // ================= STATE ===========================
// bool motorRunning = false;

// void handleSerialInput();
// void rotate60deg();
// void otherTasks();

// // ================= SETUP ===========================
// void setup() {
//   Serial.begin(115200);

//   pinMode(ENABLE_PIN, OUTPUT);
//   digitalWrite(ENABLE_PIN, LOW);       // Enable TB6600 (try HIGH if needed)

//   // IMPORTANT for TB6600 pulse timing
//   stepper.setMinPulseWidth(5);         // µs (REQUIRED)

//   stepper.setMaxSpeed(stepperSpeed);
//   stepper.setAcceleration(stepperAccel);

//   Serial.println("=== TB6600 Stepper Ready ===");
//   Serial.println("Press 'b' to rotate 60 degrees");
//   Serial.print("Microstepping (DIP): 1/");
//   Serial.println(MICROSTEP);
//   Serial.print("Steps for 60°: ");
//   Serial.println(STEPS_60_DEG);
// }

// // ================= LOOP ============================
// void loop() {
//   stepper.run();   // NON-BLOCKING

//   // Motion complete detection
//   if (motorRunning && stepper.distanceToGo() == 0) {
//     motorRunning = false;
//     Serial.println("Movement complete.");
//   }

//   handleSerialInput();
//   otherTasks();
// }

// // ================= SERIAL INPUT ====================
// void handleSerialInput() {
//   if (!Serial.available()) return;

//   char key = Serial.read();

//   if (key == 'f') {
//     if (!motorRunning) {
//         ROTATE_DEG = 240.0;
//         STEPS_PER_REV = BASE_STEPS_PER_REV * MICROSTEP;
//         STEPS_60_DEG  = (STEPS_PER_REV * ROTATE_DEG) / 360.0;
//       rotate60deg();
//     } else {
//       Serial.println("Motor busy. Wait...");
//     }
//   }
//   else if(key == 'b') {
//     if (!motorRunning) {
//         ROTATE_DEG = -240.0;
//         STEPS_PER_REV = BASE_STEPS_PER_REV * MICROSTEP;
//         STEPS_60_DEG  = (STEPS_PER_REV * ROTATE_DEG) / 360.0;
//       rotate60deg();
//     } else {
//       Serial.println("Motor busy. Wait...");
//     }
//   }
// }

// // ================= ROTATION ========================
// void rotate60deg() {
//   stepper.move(STEPS_60_DEG);   // Relative move
//   motorRunning = true;

//   Serial.print("Rotating 60° → ");
//   Serial.print(STEPS_60_DEG);
//   Serial.println(" steps");
// }

// // ================= OTHER TASKS =====================
// void otherTasks() {
//   // Place ROS2, sensors, telemetry, LEDs, etc here
// }
