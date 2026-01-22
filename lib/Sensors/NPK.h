//   /*
//   * https://www.electroniclinic.com/
//   * Soil NPK Sensor with Arduino for measuring Nitrogen, Phosphorus, and Potassium
//   * Modified for Teensy 4.1 - using Hardware Serial
//   */
//   #include <Wire.h>
  
//   // Teensy 4.1 Hardware Serial Pins:
//   // Serial1: RX=0, TX=1
//   // Serial4: RX=16, TX=17
//   // Using Serial4 for RS485 communication
//   #define RS485Serial Serial4  // Use hardware Serial4
//   #define RS485_RX 16  // Connect RS485 RO to pin 16
//   #define RS485_TX 17  // Connect RS485 DI to pin 17
//   #define RE 3
//   #define DE 4
  
//   // The following are the Inquiry frames which are send to the NPK sensor
//   //for reading the Nitrogen, Phosphorus, and Potassium values
//   // We defined three arrays with names nitro_inquiry_frame, phos_inquiry_frame, and pota_inquiry_frame 
//   // Each inquiry frame have 8 values
//   const byte nitro_inquiry_frame[] = {0x01,0x03, 0x00, 0x1e, 0x00, 0x01, 0xe4, 0x0c};
//   const byte phos_inquiry_frame[] = {0x01,0x03, 0x00, 0x1f, 0x00, 0x01, 0xb5, 0xcc};
//   const byte pota_inquiry_frame[] = {0x01,0x03, 0x00, 0x20, 0x00, 0x01, 0x85, 0xc0};
  
//   byte values[11];
//   // Using hardware Serial4 instead of SoftwareSerial for Teensy 4.1

//   byte nitrogen();
//   byte phosphorous();
//   byte potassium();

// class NPKSensor {
// public:
//     NPKSensor();
//     void begin();
//     void readNPK(byte &nitrogen, byte &phosphorus, byte &potassium);
// private:
//     byte nitrogen();
//     byte phosphorous();
//     byte potassium();
// };  
// #endif