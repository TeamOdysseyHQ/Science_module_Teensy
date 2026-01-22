// // #include <Arduino.h>
// // #include <ModbusMaster.h>
// // #include "NPK.h"

// // static ModbusMaster node;
// // static int currentDeRePin = -1;

// // static void preTransmission() {
// //   if (currentDeRePin >= 0) {
// //     digitalWrite(currentDeRePin, HIGH);
// //   }
// // }

// // static void postTransmission() {
// //   if (currentDeRePin >= 0) {
// //     digitalWrite(currentDeRePin, LOW);
// //   }
// // }

// // NPKSensor::NPKSensor(int deRePin) : _deRePin(deRePin) {}

// // void NPKSensor::begin() {
// //   Serial1.begin(9600);

// //   pinMode(_deRePin, OUTPUT);
// //   digitalWrite(_deRePin, LOW);

// //   currentDeRePin = _deRePin;
// //   node.begin(1, Serial1); // Sensor ID = 1
// //   node.preTransmission(preTransmission);
// //   node.postTransmission(postTransmission);
// // }

// // bool NPKSensor::readNPK(uint16_t &nitrogen, uint16_t &phosphorus, uint16_t &potassium) {
// //   uint8_t result = node.readHoldingRegisters(0x0000, 3);

// //   if (result == node.ku8MBSuccess) {
// //     nitrogen   = node.getResponseBuffer(0);
// //     phosphorus = node.getResponseBuffer(1);
// //     potassium  = node.getResponseBuffer(2);
// //     return true;
// //   }

// //   return false;
// // }


// /*
//  * https://www.circuitschools.com/
//  * Interfacing Soil NPK Sensor with Arduino for measuring 
//  * Nitrogen, Phosphorus, and Potassium nutrients
//  */
 
// #include <AltSoftSerial.h>
// #include "NPK.h"

// AltSoftSerial mod;
// NPKSensor::NPKSensor() {}
 
// void NPKSensor::begin() {
//   Serial.begin(4800);
//   mod.begin(4800);
//   pinMode(RE, OUTPUT);
//   pinMode(DE, OUTPUT);
 
//   // put RS-485 into receive mode
//   digitalWrite(DE, LOW);
//   digitalWrite(RE, LOW);
// }
 
// void NPKSensor::readNPK(byte &nitrogen_val, byte &phosphorus_val, byte &potassium_val) {
//   Serial.print("Nitrogen: ");
//   nitrogen_val = nitrogen();
//   Serial.print(" = ");
//   Serial.print(nitrogen_val);
//   Serial.println(" mg/kg");
//   delay(250);
 
//   Serial.print("Phosphorous: ");
//   phosphorus_val = phosphorous();
//   Serial.print(" = ");
//   Serial.print(phosphorus_val);
//   Serial.println(" mg/kg");
//   delay(250);
 
//   Serial.print("Potassium: ");
//   potassium_val = potassium();
//   Serial.print(" = ");
//   Serial.print(potassium_val);
//   Serial.println(" mg/kg");
//   Serial.println();
//   Serial.println();
//   delay(3000);
// }
 
// byte NPKSensor::nitrogen() {
//   // clear the receive buffer
//   mod.flushInput();
 
//   // switch RS-485 to transmit mode
//   digitalWrite(DE, HIGH);
//   digitalWrite(RE, HIGH);
//   delay(1);
 
//   // write out the message
//   for (uint8_t i = 0; i < sizeof(nitro); i++ ) mod.write( nitro[i] );
 
//   // wait for the transmission to complete
//   mod.flush();
  
//   // switching RS485 to receive mode
//   digitalWrite(DE, LOW);
//   digitalWrite(RE, LOW);
 
//   // delay to allow response bytes to be received!
//   delay(200);
 
//   // read in the received bytes
//   for (byte i = 0; i < 7; i++) {
//     values[i] = mod.read();
//     Serial.print(values[i], HEX);
//     Serial.print(' ');
//   }
//   return values[4];
// }
 
// byte NPKSensor::phosphorous() {
//   mod.flushInput();
//   digitalWrite(DE, HIGH);
//   digitalWrite(RE, HIGH);
//   delay(1);
//   for (uint8_t i = 0; i < sizeof(phos); i++ ) mod.write( phos[i] );
//   mod.flush();
//   digitalWrite(DE, LOW);
//   digitalWrite(RE, LOW);
// // delay to allow response bytes to be received!
//   delay(200);
//   for (byte i = 0; i < 7; i++) {
//     values[i] = mod.read();
//     Serial.print(values[i], HEX);
//     Serial.print(' ');
//   }
//   return values[4];
// }
 
// byte NPKSensor::potassium() {
//   mod.flushInput();
//   digitalWrite(DE, HIGH);
//   digitalWrite(RE, HIGH);
//   delay(1);
//   for (uint8_t i = 0; i < sizeof(pota); i++ ) mod.write( pota[i] );
//   mod.flush();
//   digitalWrite(DE, LOW);
//   digitalWrite(RE, LOW);
// // delay to allow response bytes to be received!
//   delay(200);
//   for (byte i = 0; i < 7; i++) {
//     values[i] = mod.read();
//     Serial.print(values[i], HEX);
//     Serial.print(' ');
//   }
//   return values[4];
// }
