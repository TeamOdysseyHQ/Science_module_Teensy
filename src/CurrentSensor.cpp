// #include <ACS712.h>

// #define ACS_PIN A0

// // Create ACS712 instance
// // 20A model, 3.3V supply, 12-bit ADC (4095), 100 mV/A sensitivity
// ACS712 currentSensor(ACS_PIN, 3.3, 4095, 100.0);

// void setup() {
//   Serial.begin(115200);
//   delay(2000);

//   analogReadResolution(12);
//   analogReadAveraging(16);  // good for DC stability

//   // Calibrate zero current (NO LOAD connected!)
//   currentSensor.autoMidPoint();

//   Serial.println("ACS712-20A DC Current Ready");
// }

// void loop() {
//   // Read DC current
//   float current = currentSensor.mA_DC();

//   Serial.print("DC Current: ");
//   Serial.print(current, 3);
//   Serial.println(" A");

//   delay(500);
// }
