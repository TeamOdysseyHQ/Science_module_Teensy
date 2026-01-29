/**
 * @author: Rtamanyu N J
 * @date: 2024-06-16
 * @note: Works with JXBS-3001 NPK Sensor over Modbus RTU via RS485 (the correct register: - 21,22,23)
 * 
 */
#include "NPK_ModBuster.h"
#include "CustomPrint.h"

NPK_MB_Sensor::NPK_MB_Sensor() {
    // Constructor
}

/* -------- Direction Control Functions -------- */
void preTransmission() {
  // digitalWrite(RS485_RE, HIGH);  // Disable receiver
  analogWrite(RS485_RE, 255);  // Disable receiver
  analogWrite(RS485_DE, 255);  // Enable transmitter
  // digitalWrite(RS485_DE, HIGH);  // Enable transmitter
}

void postTransmission() {
  // digitalWrite(RS485_DE, LOW);   // Disable transmitter
  analogWrite(RS485_RE, 0);   // Enable receiver
  analogWrite(RS485_DE, 0);   // Disable transmitter
  // digitalWrite(RS485_RE, LOW);   // Enable receiver
}

void NPK_MB_Sensor::begin() {
  Serial.begin(115200);
  while (!Serial) {}

  /* Serial4: RX=16 TX=17 */
  RS485Serial.begin(9600);

  pinMode(RS485_DE, OUTPUT);
  pinMode(RS485_RE, OUTPUT);

  digitalWrite(RS485_DE, LOW);  // TX OFF
  digitalWrite(RS485_RE, LOW);  // RX ON

  node_ModBuster.begin(1, RS485Serial);      // Slave ID = 1
  node_ModBuster.preTransmission(preTransmission);
  node_ModBuster.postTransmission(postTransmission);

  println("JXBS-3001 NPK Sensor Initialized");
  println("Reading registers 0x001E - 0x0020");
}

bool NPK_MB_Sensor::readNPK(uint16_t &nitrogen, uint16_t &phosphorus, uint16_t &potassium) {
  /* Read N, P, K */
  result = node_ModBuster.readHoldingRegisters(0x0021, 3);

  if (result == node_ModBuster.ku8MBSuccess) {
    nitrogen   = node_ModBuster.getResponseBuffer(0); // 0x0021
    phosphorus = node_ModBuster.getResponseBuffer(1); // 0x0022
    potassium  = node_ModBuster.getResponseBuffer(2); // 0x0023
    return true;
  }
  else {
    return false;
  }
}