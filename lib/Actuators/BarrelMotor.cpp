#include "BarrelMotor.h"
#include "CustomPrint.h"


// ================= PIN DEFINITIONS =================
// #define STEP_PIN    7 (TB6600) - 10 (B126)
// #define DIR_PIN     8 (TB6600) - 11 (B126)
// #define ENABLE_PIN  9 (TB6600) - 12 (B126)

BarrelMotor::BarrelMotor(int step_pin, int dir_pin, int en_pin, bool isTB6600)
    : STEP_PIN(step_pin), DIR_PIN(dir_pin), ENABLE_PIN(en_pin), isTB6600(isTB6600) {}

// ================= SETUP ===========================
void BarrelMotor::setup() {
  pinMode(ENABLE_PIN, OUTPUT);
  if (isTB6600)
   digitalWrite(ENABLE_PIN, LOW);       // Enable TB6600 
  else
   digitalWrite(ENABLE_PIN, HIGH);       // Enable B126

  // IMPORTANT for TB6600 pulse timing
  stepper.setMinPulseWidth(5);         // µs (REQUIRED)

  stepper.setMaxSpeed(stepperSpeed);
  stepper.setAcceleration(stepperAccel);

  println("=== TB6600 Stepper Ready ===");
  println("Press 'f' to rotate 240 forward degrees");
  println("Press 'b' to rotate 240 backward degrees");
  print("Microstepping (DIP): 1/");
  println(MICROSTEP);
  print("Steps for 240°: ");
  println(STEPS_240_DEG);
}

// ================= LOOP ============================
void BarrelMotor::run() {
  stepper.run();   // NON-BLOCKING

  // Motion complete detection
  if (motorRunning && stepper.distanceToGo() == 0) {
    motorRunning = false;
    println("Movement complete.");
  }
}

// ================= SERIAL INPUT ====================
void BarrelMotor::handleBarellRotateInput(bool forward) {
  if (!motorRunning) {
    rotate240deg(forward);
  } else {
    println("Motor busy. Wait...");
  }
}

// ================= ROTATION ========================
void BarrelMotor::rotate240deg(bool forward) {
  motorRunning = true;

  if (forward)
  {
    stepper.move(STEPS_240_DEG);
    print("Rotating 240° → ");
  }
  else {
    stepper.move(-STEPS_240_DEG);
    print("Rotating 240° ← ");
  }

  print(STEPS_240_DEG);
  println(" steps");
}