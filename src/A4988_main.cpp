#include <Arduino.h>

// ================== A4988 PIN DEFINITIONS ==================
#define STEP_PIN 7
#define DIR_PIN 8
#define MS1_PIN 8
#define MS2_PIN 9
#define MS3_PIN 10
#define ENABLE_PIN 11

// ================== MOTOR PARAMETERS ==================
const int stepsPerMove = 100;   // n steps per arrow press
const int stepDelayMicros = 800; // speed control

// ================== MICROSTEP MODES ==================
enum MicrostepMode {
  FULL,
  MICRO_1_8,
  MICRO_1_16,
  MICRO_1_32
};

MicrostepMode currentMicrostep = FULL;

// ================== SET MICROSTEPPING ==================
void setMicrostepping(MicrostepMode mode) {
  switch (mode) {
    case FULL:
      digitalWrite(MS1_PIN, LOW);
      digitalWrite(MS2_PIN, LOW);
      digitalWrite(MS3_PIN, LOW);
      Serial.println("Microstep: FULL (1/1)");
      break;

    case MICRO_1_8:
      digitalWrite(MS1_PIN, HIGH);
      digitalWrite(MS2_PIN, HIGH);
      digitalWrite(MS3_PIN, LOW);
      Serial.println("Microstep: 1/8");
      break;

    case MICRO_1_16:
      digitalWrite(MS1_PIN, HIGH);
      digitalWrite(MS2_PIN, HIGH);
      digitalWrite(MS3_PIN, HIGH);
      Serial.println("Microstep: 1/16");
      break;

    case MICRO_1_32:
      digitalWrite(MS1_PIN, LOW);
      digitalWrite(MS2_PIN, HIGH);
      digitalWrite(MS3_PIN, HIGH);
      Serial.println("Microstep: 1/32");
      break;
  }
}

// ================== ROTATE MOTOR ==================
void rotateMotor(bool clockwise, int steps) {
  digitalWrite(DIR_PIN, clockwise ? HIGH : LOW);

  for (int i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(stepDelayMicros);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(stepDelayMicros);
  }
}

// // ================== SETUP ==================
// void setup() {
//   Serial.begin(115200);

//   while (!Serial) {
//     ; // Wait for serial monitor to connect
//   }
  

//   pinMode(STEP_PIN, OUTPUT);
//   pinMode(DIR_PIN, OUTPUT);
//   pinMode(MS1_PIN, OUTPUT);
//   pinMode(MS2_PIN, OUTPUT);
//   pinMode(MS3_PIN, OUTPUT);
//   pinMode(ENABLE_PIN, OUTPUT);

//   digitalWrite(ENABLE_PIN, LOW); // Enable driver
//   setMicrostepping(FULL);

//   Serial.println("=== A4988 Keyboard Control Ready ===");
//   Serial.println("↑ : Clockwise");
//   Serial.println("↓ : Anti-clockwise");
//   Serial.println("0: Full | 1: 1/8 | 2: 1/16 | 3: 1/32");
// }

// // ================== LOOP ==================
// void loop() {
//   static uint8_t escState = 0;

//   if (Serial.available()) {
//     char c = Serial.read();

//     // --------- Arrow Key Handling (ESC sequences) ---------
//     if (escState == 0 && c == 27) { // ESC
//       escState = 1;
//       return;
//     }

//     if (escState == 1 && c == '[') {
//       escState = 2;
//       return;
//     }

//     if (escState == 2) {
//       escState = 0;

//       if (c == 'A') { // Up Arrow
//         Serial.println("Rotate CW");
//         rotateMotor(true, stepsPerMove);
//       }
//       else if (c == 'B') { // Down Arrow
//         Serial.println("Rotate CCW");
//         rotateMotor(false, stepsPerMove);
//       }
//       return;
//     }

//     // --------- Microstep Selection ---------
//     switch (c) {
//       case '0':
//         currentMicrostep = FULL;
//         break;
//       case '1':
//         currentMicrostep = MICRO_1_8;
//         break;
//       case '2':
//         currentMicrostep = MICRO_1_16;
//         break;
//       case '3':
//         currentMicrostep = MICRO_1_32;
//         break;
//       default:
//         return;
//     }

//     setMicrostepping(currentMicrostep);
//   }
// }
