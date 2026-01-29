// #include "LinearActuator.h"
// #include "CustomPrint.h"

// LinearActuator::LinearActuator(int stepPin, int dirPin, int ms1Pin, int ms2Pin, int ms3Pin, int enablePin)
//     : STEP_PIN(stepPin), DIR_PIN(dirPin), MS1_PIN(ms1Pin), MS2_PIN(ms2Pin), MS3_PIN(ms3Pin), ENABLE_PIN(enablePin) {}



// // ================== SET MICROSTEPPING ==================
// void LinearActuator::setMicrostepping(MicrostepMode mode) {
//   currentMicrostep = mode;
//   switch (mode) {
//     case FULL:
//       digitalWrite(MS1_PIN, LOW);
//       digitalWrite(MS2_PIN, LOW);
//       digitalWrite(MS3_PIN, LOW);
//       println("LA Microstep: FULL (1/1)");
//       break;

//     case MICRO_1_4:
//       digitalWrite(MS1_PIN, LOW);
//       digitalWrite(MS2_PIN, HIGH);
//       digitalWrite(MS3_PIN, LOW);
//       println("LA Microstep: 1/4");
//       break;

//     case MICRO_1_8:
//       digitalWrite(MS1_PIN, HIGH);
//       digitalWrite(MS2_PIN, HIGH);
//       digitalWrite(MS3_PIN, LOW);
//       println("LA Microstep: 1/8");
//       break;

//     case MICRO_1_16:
//       digitalWrite(MS1_PIN, HIGH);
//       digitalWrite(MS2_PIN, HIGH);
//       digitalWrite(MS3_PIN, HIGH);
//       println("LA Microstep: 1/16");
//       break;
//   }
// }

// // ================== ROTATE MOTOR ==================
// void LinearActuator::rotateMotor(bool clockwise, int steps) {
//   steps *= static_cast<int>(currentMicrostep); // adjust for microstepping
//   digitalWrite(DIR_PIN, clockwise ? HIGH : LOW);

//   for (int i = 0; i < steps; i++) {
//     digitalWrite(STEP_PIN, HIGH);
//     delayMicroseconds(stepDelayMicros);
//     digitalWrite(STEP_PIN, LOW);
//     delayMicroseconds(stepDelayMicros);
//   }
// }

// // ================== SETUP ==================
// void LinearActuator::setup() {
//   pinMode(STEP_PIN, OUTPUT);
//   pinMode(DIR_PIN, OUTPUT);
//   pinMode(MS1_PIN, OUTPUT);
//   pinMode(MS2_PIN, OUTPUT);
//   pinMode(MS3_PIN, OUTPUT);
//   pinMode(ENABLE_PIN, OUTPUT);

//   digitalWrite(ENABLE_PIN, LOW); // Enable driver
//   setMicrostepping(FULL);
//   println("=== Linear Actuator Initialized ===");
// }

#include "LinearActuator.h"
#include "CustomPrint.h"


/* ================= TB6600 PIN DEFINITIONS ================= */
// #define STEP_PIN   2   // PUL+
// #define DIR_PIN    3   // DIR+
// #define ENABLE_PIN 4   // ENA+

LinearActuator::LinearActuator(int step_pin, int dir_pin, int en_pin, bool isTB6600) : STEP_PIN(step_pin), DIR_PIN(dir_pin), EN_PIN(en_pin), isTB6600(isTB6600) {}


/* ================= MOVE MOTOR ================= */
void LinearActuator::moveMotor(bool anticlockwise)
{
  long steps = n_revolutions *
               fullStepsPerRev *
               microstepMultiplier;

  if (!anticlockwise)
    steps = -steps;

  stepper.move(steps);
  motorRunning = true;

  println(anticlockwise ?
    "Rotating ANTICLOCKWISE" :
    "Rotating CLOCKWISE");
}

/* ================= SETUP ================= */
void LinearActuator::setup()
{
  pinMode(EN_PIN, OUTPUT);
  // most of the time enabled pin won't be connected to HIGH or LOW directly
  if(isTB6600)
   digitalWrite(EN_PIN, LOW);   // Enable TB6600 
  else
   digitalWrite(EN_PIN, HIGH);   // Enable B126

  stepper.setMaxSpeed(motorSpeed);
  stepper.setAcceleration(motorAccel);

  println("TB6600 Stepper Ready");
  print("Microstep multiplier: ");
  println(microstepMultiplier);
  println("UP    : Anticlockwise");
  println("DOWN  : Clockwise");
}

void LinearActuator::run()
{
  stepper.run();

  // Detect motion completion
  if (motorRunning && stepper.distanceToGo() == 0)
  {
    motorRunning = false;
    println("Motion Complete");
  }
}

/* ================= LOOP ================= */
// void loop()
// {
  

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
