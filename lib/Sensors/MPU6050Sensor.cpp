#include "MPU6050Sensor.h"
#include "CustomPrint.h"

MPU6050_Sensor::MPU6050_Sensor() {}

void MPU6050_Sensor::begin() {
    Wire.begin();
    mpu.initialize();
    if (mpu.testConnection()) {
        println("MPU6050 connection successful");
    } else {
        println("MPU6050 connection failed");
    }
}

bool MPU6050_Sensor::testConnection() {
    return mpu.testConnection();
}

void MPU6050_Sensor::getAcceleration(int16_t* ax, int16_t* ay, int16_t* az) {
    mpu.getAcceleration(ax, ay, az);
}

void MPU6050_Sensor::getRotation(int16_t* gx, int16_t* gy, int16_t* gz) {
    mpu.getRotation(gx, gy, gz);
}
