#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

#define BMP280_ADDRESS 0x76   // Try 0x77 if this fails

Adafruit_BMP280 bmp;  // I2C object

// void setup() {
//   Serial.begin(115200);
//   while (!Serial) delay(10);

//   Serial.println("BMP280 Test");

//   if (!bmp.begin(BMP280_ADDRESS)) {
//     Serial.println("❌ BMP280 not found!");
//     while (1);
//   }

//   // Optional: configure sensor
//   bmp.setSampling(
//     Adafruit_BMP280::MODE_NORMAL,
//     Adafruit_BMP280::SAMPLING_X2,    // Temp
//     Adafruit_BMP280::SAMPLING_X16,   // Pressure
//     Adafruit_BMP280::FILTER_X16,
//     Adafruit_BMP280::STANDBY_MS_500
//   );

//   Serial.println("✅ BMP280 initialized");
// }

// void loop() {
//   Serial.print("Temperature = ");
//   Serial.print(bmp.readTemperature());
//   Serial.println(" °C");

//   Serial.print("Pressure = ");
//   Serial.print(bmp.readPressure() / 100.0F);
//   Serial.println(" hPa");

//   Serial.print("Altitude = ");
//   Serial.print(bmp.readAltitude(1013.25));  // Sea-level pressure
//   Serial.println(" m");

//   Serial.println("------------------------");
//   delay(1000);
// }
