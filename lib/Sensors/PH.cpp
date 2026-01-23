// filepath: d:/Amrita/Mars_Rover/Component_testing/lib/Sensors/PH.cpp
#include "PH.h"
#include "CustomPrint.h"

PHSensor::PHSensor(int phPin) : _phPin(phPin) {}

void PHSensor::begin() {
  analogReadResolution(12); // Teensy ADC: 12-bit (0–4095)
}

float PHSensor::readPH() {
  int raw = analogRead(_phPin);
  float voltage = (raw * 3.3f) / 4095.0f; // assuming 3.3V ADC reference
  float pH = 7.0f + (voltage - _ph7Voltage) * _phSlope;

  println("pH Voltage: " + String(voltage, 3) + " V | pH Value: " + String(pH, 2));
  return pH;
}
