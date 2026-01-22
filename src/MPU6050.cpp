#include <Wire.h>
#include <Arduino.h>
#include <MPU6050.h>

MPU6050 mpu;

// void setup() {
//   Serial.begin(115200);
//   Wire.begin(); // SDA=18, SCL=19

//   mpu.initialize();

//   if (mpu.testConnection()) {
//     Serial.println("MPU6050 connected");
//   } else {
//     Serial.println("MPU6050 connection failed");
//   }
// }

// void loop() {
//   int16_t ax, ay, az;
//   int16_t gx, gy, gz;

//   mpu.getAcceleration(&ax, &ay, &az);
//   mpu.getRotation(&gx, &gy, &gz);

//   Serial.print("Acc: ");
//   Serial.print(ax); Serial.print(", ");
//   Serial.print(ay); Serial.print(", ");
//   Serial.println(az);

//   Serial.print("Gyro: ");
//   Serial.print(gx); Serial.print(", ");
//   Serial.print(gy); Serial.print(", ");
//   Serial.println(gz);

//   delay(500);
// }
