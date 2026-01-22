#include <Arduino.h>

// ===== TCS3200 Pins =====
#define S0 2
#define S1 3
#define S2 4
#define S3 5
#define OUT_PIN 6

unsigned long redFreq, greenFreq, blueFreq;

// void setup() {
//   Serial.begin(115200);

//   pinMode(S0, OUTPUT);
//   pinMode(S1, OUTPUT);
//   pinMode(S2, OUTPUT);
//   pinMode(S3, OUTPUT);
//   pinMode(OUT_PIN, INPUT);

//   // Set frequency scaling to 20%
//   digitalWrite(S0, HIGH);
//   digitalWrite(S1, LOW);

//   Serial.println("TCS3200 Violet Color Detection Started");
// }

// unsigned long readColor(bool s2, bool s3) {
//   digitalWrite(S2, s2);
//   digitalWrite(S3, s3);
//   delay(5);  // settle time
//   return pulseIn(OUT_PIN, LOW);
// }

// void loop() {
//   // Read Red
//   redFreq = readColor(LOW, LOW);

//   // Read Blue
//   blueFreq = readColor(LOW, HIGH);

//   // Read Green
//   greenFreq = readColor(HIGH, HIGH);

//   Serial.print("R: "); Serial.print(redFreq);
//   Serial.print(" G: "); Serial.print(greenFreq);
//   Serial.print(" B: "); Serial.println(blueFreq);

//   // ===== VIOLET DETECTION LOGIC =====
//   // These values MUST be tuned after calibration
//   if (
//     blueFreq < 150 &&      // High blue intensity
//     redFreq < 200 &&       // Moderate red
//     greenFreq > 250        // Low green
//   ) {
//     Serial.println("🎨 Violet Color Detected");
//   } else {
//     Serial.println("Color Not Violet");
//   }

//   Serial.println("--------------------");
//   delay(500);
// }
