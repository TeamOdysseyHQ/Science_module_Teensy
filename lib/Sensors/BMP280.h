#ifndef BMP280_H
#define BMP280_H

#include <Wire.h>
#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

#define SEALEVELPRESSURE_HPA 1013.25  // Adjust if needed
#define BMP280_ADDRESS 0x76   // Try 0x77 if this fails


class BMP280_Sensor {
public:
    BMP280_Sensor();
    void begin();
    float readTemperature();
    float readPressure();
    float readAltitude();
private:
    Adafruit_BMP280 bmp; // I2C object
};

#endif