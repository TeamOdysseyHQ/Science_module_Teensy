// #include <AccelStepper.h>

// /* ================= PIN DEFINITIONS ================= */
// #define STEP_PIN    19
// #define DIR_PIN     18
// #define MS1_PIN     4
// #define MS2_PIN     5
// #define MS3_PIN     6
// #define ENABLE_PIN  7   // optional but recommended

// /* ================= STEPPER SETUP ================= */
// AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// /* ================= USER VARIABLES ================= */
// float n_revolutions = 2.0;      // <<< revolutions per move
// float motorSpeed   = 1200.0;   // <<< steps/sec
// float motorAccel   = 2000.0;   // <<< acceleration

// const int fullStepsPerRev = 200; // NEMA17 (1.8°)

// /* ================= STATE ================= */
// bool motorRunning = false;
// int microstepFactor = 1;

// /* ================= MICROSTEPPING (DRV8825) ================= */
// void setMicrostepping(int mode)
// {
//   switch (mode)
//   {
//     case 0: // Full
//       digitalWrite(MS1_PIN, LOW);
//       digitalWrite(MS2_PIN, LOW);
//       digitalWrite(MS3_PIN, LOW);
//       microstepFactor = 1;
//       break;

//     case 1: // 1/4
//       digitalWrite(MS1_PIN, LOW);
//       digitalWrite(MS2_PIN, HIGH);
//       digitalWrite(MS3_PIN, LOW);
//       microstepFactor = 4;
//       break;

//     case 2: // 1/8
//       digitalWrite(MS1_PIN, HIGH);
//       digitalWrite(MS2_PIN, HIGH);
//       digitalWrite(MS3_PIN, LOW);
//       microstepFactor = 8;
//       break;

//     case 3: // 1/32 (DRV8825)
//       digitalWrite(MS1_PIN, HIGH);
//       digitalWrite(MS2_PIN, LOW);
//       digitalWrite(MS3_PIN, HIGH);
//       microstepFactor = 32;
//       break;
//   }

//   Serial.print("Microstepping set to 1/");
//   Serial.println(microstepFactor);
// }

// /* ================= MOTOR MOVE ================= */
// void moveMotor(bool anticlockwise)
// {
//   long steps =
//     n_revolutions * fullStepsPerRev * microstepFactor;

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

//   pinMode(MS1_PIN, OUTPUT);
//   pinMode(MS2_PIN, OUTPUT);
//   pinMode(MS3_PIN, OUTPUT);
//   pinMode(ENABLE_PIN, OUTPUT);

//   digitalWrite(ENABLE_PIN, LOW); // enable DRV8825

//   setMicrostepping(0); // default full step

//   stepper.setMaxSpeed(motorSpeed);
//   stepper.setAcceleration(motorAccel);

//   Serial.println("DRV8825 Stepper Ready");
//   Serial.println("UP    : Anticlockwise");
//   Serial.println("DOWN  : Clockwise");
//   Serial.println("0     : Full step");
//   Serial.println("1     : 1/4 step");
//   Serial.println("2     : 1/8 step");
//   Serial.println("3     : 1/32 step");
// }

// /* ================= LOOP ================= */
// void loop()
// {
//   stepper.run();

//   // motion finished
//   if (motorRunning && stepper.distanceToGo() == 0)
//   {
//     motorRunning = false;
//     Serial.println("Motion Complete");
//   }

//   // block arrow keys while moving
//   if (motorRunning)
//     return;

//   if (Serial.available())
//   {
//     char c = Serial.read();

//     // Arrow keys (ESC [ A/B)
//     if (c == 27 && Serial.available() >= 2)
//     {
//       Serial.read();          // '['
//       char arrow = Serial.read();

//       if (arrow == 'A')       // UP
//         moveMotor(true);
//       else if (arrow == 'B')  // DOWN
//         moveMotor(false);
//     }

//     // Microstepping keys
//     if (c >= '0' && c <= '3')
//       setMicrostepping(c - '0');
//   }
// }

// #include <Arduino.h>
// #define STEP_PIN 18
// #define DIR_PIN 19
// #define ENABLE_PIN 4

// const int stepDelay = 1000; // Speed control (microseconds)

// void setup() {
//   pinMode(STEP_PIN, OUTPUT);
//   pinMode(DIR_PIN, OUTPUT);
//   pinMode(ENABLE_PIN, OUTPUT);
  
//   digitalWrite(ENABLE_PIN, LOW);  // Enable the motor
//   digitalWrite(DIR_PIN, HIGH);    // Set direction (HIGH = clockwise)
// }

// void loop() {
// //   Serial.println("Stepping motor one step");
//   digitalWrite(STEP_PIN, HIGH);
//   delayMicroseconds(stepDelay);
//   digitalWrite(STEP_PIN, LOW);
//   delayMicroseconds(stepDelay);
// }
