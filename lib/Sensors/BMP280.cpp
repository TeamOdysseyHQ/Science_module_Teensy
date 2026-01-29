#include "BMP280.h"
#include "CustomPrint.h"

BMP280_Sensor::BMP280_Sensor() : bmp() {}

void BMP280_Sensor::begin() {
  bmp.begin(BMP280_ADDRESS);

  // Wire2.begin();
  Wire.setSDA(25);   // SDA pin
  Wire.setSCL(24);   // SCL pin
  Wire.begin();

  if (!bmp.begin(BMP280_ADDRESS)) {
    println("BMP280 not found!");
    while (1);
  }

  // Optional: configure sensor
  bmp.setSampling(
    Adafruit_BMP280::MODE_NORMAL,
    Adafruit_BMP280::SAMPLING_X2,    // Temp
    Adafruit_BMP280::SAMPLING_X16,   // Pressure
    Adafruit_BMP280::FILTER_X16,
    Adafruit_BMP280::STANDBY_MS_500
  );

  println("BMP280 initialized");
}

float BMP280_Sensor::readTemperature() {
  return bmp.readTemperature();
}

float BMP280_Sensor::readPressure() {
  return bmp.readPressure();
}

float BMP280_Sensor::readAltitude() {
  return bmp.readAltitude(SEALEVELPRESSURE_HPA);  // Sea-level pressure
}
