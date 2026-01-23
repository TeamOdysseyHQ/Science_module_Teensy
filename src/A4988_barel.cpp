#include <Arduino.h>

// Pin definitions
#define STEP_PIN 2
#define DIR_PIN  3
#define MS1_PIN  4
#define MS2_PIN  5
#define MS3_PIN  6
#define EN_PIN   7   // Optional (LOW = enabled)

// Motor parameters
const int STEPS_PER_REV = 200;   // 1.8° motor
const float ROTATE_DEG = 60.0;   // rotation angle

// Microstep mode
// 0 = Full
// 1 = 1/8
// 2 = 1/16
// 3 = 1/32 (NOT supported by A4988 → fallback to 1/16)
int microstepMode = 1;

// Derived
int microstepMultiplier = 1;

void setMicrostepping(int mode) {
  switch (mode) {
    case 0: // Full step
      digitalWrite(MS1_PIN, LOW);
      digitalWrite(MS2_PIN, LOW);
      digitalWrite(MS3_PIN, LOW);
      microstepMultiplier = 1;
      Serial.println("Microstep: FULL");
      break;

    case 1: // 1/4 step
      digitalWrite(MS1_PIN, LOW);
      digitalWrite(MS2_PIN, HIGH);
      digitalWrite(MS3_PIN, LOW);
      microstepMultiplier = 4;
      Serial.println("Microstep: 1/4");
      break;

    case 2: // 1/8 step
      digitalWrite(MS1_PIN, HIGH);
      digitalWrite(MS2_PIN, HIGH);
      digitalWrite(MS3_PIN, LOW);
      microstepMultiplier = 8;
      Serial.println("Microstep: 1/8");
      break;

    case 3: // 1/16
      Serial.println("Using 1/16");
      digitalWrite(MS1_PIN, HIGH);
      digitalWrite(MS2_PIN, HIGH);
      digitalWrite(MS3_PIN, HIGH);
      microstepMultiplier = 16;
      break;
  }
}

void rotateDegrees(float degrees) {
  long steps = (STEPS_PER_REV * microstepMultiplier * degrees) / 360.0;

  digitalWrite(DIR_PIN, HIGH); // change direction if needed

  for (long i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(800);   // speed control
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(800);
  }
}

// void setup() {
//   pinMode(STEP_PIN, OUTPUT);
//   pinMode(DIR_PIN, OUTPUT);
//   pinMode(MS1_PIN, OUTPUT);
//   pinMode(MS2_PIN, OUTPUT);
//   pinMode(MS3_PIN, OUTPUT);
//   pinMode(EN_PIN, OUTPUT);

//   digitalWrite(EN_PIN, LOW); // Enable driver

//   Serial.begin(115200);
//   while (!Serial);

//   setMicrostepping(microstepMode);

//   Serial.println("Press 'b' to rotate 60 degrees");
// }

// void loop() {
//   if (Serial.available()) {
//     char key = Serial.read();

//     if (key == 'b' || key == 'B') {
//       Serial.println("Rotating 60 degrees...");
//       rotateDegrees(ROTATE_DEG);
//       Serial.println("Done");
//     }
//   }
// }
