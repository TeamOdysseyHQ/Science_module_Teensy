#ifndef VL53L0X_H
#define VL53L0X_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

class VL53L0X_Sensor {
public:
    VL53L0X_Sensor();
    bool begin();
    uint16_t readDistance();
private:
    Adafruit_VL53L0X lox = Adafruit_VL53L0X();
};

#endif