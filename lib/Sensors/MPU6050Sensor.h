#ifndef MPU6050Sensor_H
#define MPU6050Sensor_H

#include <Wire.h>
#include <Arduino.h>
#include <MPU6050.h>

typedef short int16_t;

class MPU6050_Sensor {
public:
    MPU6050_Sensor();
    void begin();
    void getAcceleration(int16_t* ax, int16_t* ay, int16_t* az);
    void getRotation(int16_t* gx, int16_t* gy, int16_t* gz);
    bool testConnection();
private:
    MPU6050 mpu;
};

#endif