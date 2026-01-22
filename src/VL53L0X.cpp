#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// void setup() {
//   Serial.begin(115200);
//   while (!Serial) delay(10);

//   // Initialize I2C
//   Wire.begin(); // SDA=18, SCL=19

//   Serial.println("VL53L0X Test");

//   if (!lox.begin()) {
//     Serial.println("Failed to boot VL53L0X");
//     while (1);
//   }

//   Serial.println("VL53L0X ready!");
// }

// void loop() {
//   VL53L0X_RangingMeasurementData_t measure;

//   lox.rangingTest(&measure, false);

//   if (measure.RangeStatus != 4) {  
//     Serial.print("Distance: ");
//     Serial.print(measure.RangeMilliMeter);
//     Serial.println(" mm");
//   } else {
//     Serial.println("Out of range");
//   }

//   delay(200);
// }
