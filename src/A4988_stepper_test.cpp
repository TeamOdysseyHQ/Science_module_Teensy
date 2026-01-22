
#include <Arduino.h>
#include <AccelStepper.h>

// Pin configuration - adjust to match your wiring
const uint8_t STEP_PIN = 7;    // step pulse
const uint8_t DIR_PIN  = 8;    // direction
const uint8_t ENABLE_PIN = 11;  // driver enable (active LOW on many drivers)

// Stepper configuration
const long STEPS_PER_REV = 200; // base steps for 1.8° motor (update if microstepping differs)
const long STEPS_90 = STEPS_PER_REV / 4;

// AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// void setup() {
//     Serial.begin(115200);
//     while (!Serial && millis() < 3000);
//   pinMode(ENABLE_PIN, OUTPUT);
//   pinMode(STEP_PIN, OUTPUT);
//   pinMode(DIR_PIN, OUTPUT);
//   digitalWrite(ENABLE_PIN, LOW); // enable driver

//   stepper.setMaxSpeed(800.0);      // steps per second
//   stepper.setAcceleration(400.0);  // steps per second^2
// }

// void loop() {
//   // Rotate +90°
//     Serial.println("Rotating +90 degrees");
//   stepper.moveTo(STEPS_90);
//   stepper.runToPosition();

//   delay(1000);

//   // Rotate -90° back to start
//     Serial.println("Rotating -90 degrees");
//   stepper.moveTo(0);
//   stepper.runToPosition();

//   // Optional pause before repeating
//   delay(500);
// }


