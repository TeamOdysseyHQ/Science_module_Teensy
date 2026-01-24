#include "CurrentSensor.h"
#include "CustomPrint.h"


CurrentSensor::CurrentSensor(int pin)
    : ACS_PIN(pin), currentSensor(pin, VREF, ADC_MAX, SENSITIVITY)
{}

void CurrentSensor::begin() {
    analogReadResolution(12);
    analogReadAveraging(16);  // good for DC stability

    println("ACS712-20A DC Current Ready");
}

void CurrentSensor::calibrateZeroCurrent() {
    // Calibrate zero current (NO LOAD connected!)
    currentSensor.autoMidPoint();
    println("ACS712-20A DC Calibirated");
}

float CurrentSensor::readCurrent() {
    // Read DC current
    return currentSensor.mA_DC();
}
