#include <Arduino.h>

#define PH_PIN A0

// Calibration values (adjust after calibration)
float PH_7_VOLTAGE = 3.3;   // voltage at pH 7
// float PH_SLOPE = -5.7;       // typical slope (~ -5.7 pH/V)
float PH_SLOPE = -1.5;       // adjusted slope for better accuracy

// void setup() {
//   Serial.begin(9600);
//   analogReadResolution(12); // Teensy ADC: 12-bit (0–4095)
// }

// void loop() {
//   int raw = analogRead(PH_PIN);
//   float voltage = (raw * 3.3) / 4095.0;

//   float pH = 7.0 + (voltage - PH_7_VOLTAGE) * PH_SLOPE;

//   Serial.print("pH Voltage: ");
//   Serial.print(voltage, 3);
//   Serial.print(" V | pH Value: ");
//   Serial.println(pH, 2);

//   delay(500);
// }