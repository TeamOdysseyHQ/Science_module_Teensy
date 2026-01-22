#include "VL53L0X.h"

VL53L0X_Sensor::VL53L0X_Sensor() {}

bool VL53L0X_Sensor::begin() {
  // Initialize I2C
  Wire.begin(); // SDA=18, SCL=19

  Serial.println("VL53L0X Test");

  if (!lox.begin()) {
    Serial.println("Failed to boot VL53L0X");
    // while (1);
    return false;
  }

  Serial.println("VL53L0X ready!");
  return true;
}

uint16_t VL53L0X_Sensor::readDistance() {
  VL53L0X_RangingMeasurementData_t measure;

  lox.rangingTest(&measure, false);

  if (measure.RangeStatus != 4) {  
    uint16_t distance = measure.RangeMilliMeter;
    return distance;
  } else {
    return 0xFFFF; // Indicate out of range
  }

  delay(200);
}
