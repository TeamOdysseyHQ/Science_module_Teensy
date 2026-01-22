#include <Arduino.h>

// // together using I2c SCL - , SDA-
// #include "MPU6050Sensor.h"
// #include "VL53L0X.h"
// #include "BMP280.h"

// MPU6050_Sensor mpu_sensor;
// VL53L0X_Sensor vl53l0x;
// BMP280_Sensor bmp_sensor;


// uint16_t distance;

// void setup() {
//   Serial.begin(115200);
//   mpu_sensor.begin();
//   vl53l0x.begin();
//   bmp_sensor.begin();
// }

// void loop() {
//   int16_t ax, ay, az;
//   int16_t gx, gy, gz;
//   mpu_sensor.getAcceleration(&ax, &ay, &az);
//   mpu_sensor.getRotation(&gx, &gy, &gz);

//   Serial.println("---------------------------");
//   Serial.print("Acc: ");
//   Serial.print(ax); Serial.print(", ");
//   Serial.print(ay); Serial.print(", ");
//   Serial.println(az);

//   Serial.print("Gyro: ");
//   Serial.print(gx); Serial.print(", ");
//   Serial.print(gy); Serial.print(", ");
//   Serial.println(gz);

//   distance = vl53l0x.readDistance();
//   if (distance != 0xFFFF) {
//     Serial.print("Distance: ");
//     Serial.print(distance);
//     Serial.println(" mm");
//   } else {
//     Serial.println("Out of range");
//   }

//   Serial.print("Temperature: ");
//   Serial.print(bmp_sensor.readTemperature());
//   Serial.println(" °C");

//   Serial.print("Pressure: ");
//   Serial.print(bmp_sensor.readPressure());
//   Serial.println(" hPa");

//   Serial.print("Approx Altitude: ");
//   Serial.print(bmp_sensor.readAltitude());
//   Serial.println(" m");

//   Serial.println("---------------------------");

//   delay(1000);
// }

// #include "GasSensors.h"

// // MQ2Sensor mq2(A4);
// // MQ4Sensor mq4(A4);
// // MQ6Sensor mq6(A2);
// MQ135Sensor mq135(A3);

// void setup() {
//   Serial.begin(9600);
//   while (!Serial);

//   analogReadResolution(10);

// //   mq2.begin();
// //   mq4.begin();
// //   mq6.begin();
//     mq135.begin();

//   Serial.println("Calibrating sensors in clean air... for 20 seconds");
//   delay(20000); // Preheat

// //   mq2.calibrate();
// //   mq4.calibrate();
// //   mq6.calibrate();
//     mq135.calibrate();

//   Serial.println("Calibration complete");
// }

// void loop() {
// //   Serial.print("MQ-2 Propane: "); Serial.print(mq2.readPropane()); Serial.println(" ppm");
// //   Serial.print("MQ-2 CO: "); Serial.print(mq2.readCO()); Serial.println(" ppm");
// //   Serial.print("MQ-2 Smoke: "); Serial.print(mq2.readSmoke()); Serial.println(" ppm");

// //   Serial.print("MQ-4 Methane: "); Serial.print(mq4.readMethane()); Serial.println(" ppm");

// //   Serial.print("MQ-6 Propane: "); Serial.print(mq6.readPropane()); Serial.println(" ppm");
// //   Serial.print("MQ-6 Butane: "); Serial.print(mq6.readButane()); Serial.println(" ppm");

//     Serial.print("MQ-135 CO2: "); Serial.print(mq135.readCO2()); Serial.println(" ppm");

//   Serial.println("----------------------------------");
//   delay(3000);
// }

// #include "PH.h"

// PHSensor ph_sensor(A0);

// void setup() {
//   Serial.begin(115200);
//   while (!Serial) delay(10);
//   ph_sensor.begin();
// }

// void loop() {
//   float pH = ph_sensor.readPH();
//   Serial.print("pH Value: ");
//   Serial.println(pH, 2);
//   delay(500);
// }

