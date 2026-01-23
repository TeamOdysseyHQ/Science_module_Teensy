#include <Arduino.h>
#include "TCS3200.h"

// ===== TCS3200 Pins =====
// #define S0 2
// #define S1 3
// #define S2 4
// #define S3 5
// #define OUT_PIN 6

TCS3200_Sensor::TCS3200_Sensor(int s0, int s1, int s2, int s3, int outPin) {
    this->s0 = s0;
    this->s1 = s1;
    this->s2 = s2;
    this->s3 = s3;
    this->outPin = outPin;
}


void TCS3200_Sensor::begin() {
  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);
  pinMode(outPin, INPUT);

  // Set frequency scaling to 20%
  digitalWrite(s0, HIGH);
  digitalWrite(s1, LOW);
  Serial.println("TCS3200 Violet Color Detection Started");
}

unsigned long TCS3200_Sensor::readColor(bool s2, bool s3) {
  digitalWrite(this->s2, s2);
  digitalWrite(this->s3, s3);
  delay(5);  // settle time
  return pulseIn(outPin, LOW);
}

bool TCS3200_Sensor::isColorViolet() {
  // Read Red
  redFreq = readColor(LOW, LOW);

  // Read Blue
  blueFreq = readColor(LOW, HIGH);

  // Read Green
  greenFreq = readColor(HIGH, HIGH);

  Serial.print("R: "); Serial.print(redFreq);
  Serial.print(" G: "); Serial.print(greenFreq);
  Serial.print(" B: "); Serial.println(blueFreq);

  // ===== VIOLET DETECTION LOGIC =====
  // These values MUST be tuned after calibration
  if (
    blueFreq < 150 &&      // High blue intensity
    redFreq < 200 &&       // Moderate red
    greenFreq > 250        // Low green
  ) {
    Serial.println("🎨 Violet Color Detected");
    Serial.println("--------------------");
    return true;
  } else {
    Serial.println("Color Not Violet");
    Serial.println("--------------------");
    return false;
  }
}