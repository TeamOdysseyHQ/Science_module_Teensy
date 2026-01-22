/**
 * @author: Rtamanyu N J
 * @date: 2024-06-16
 * @note: Works with JXBS-3001 NPK Sensor over Modbus RTU via RS485 (the correct register: - 21,22,23)
 * 
 */

// #include <ModbusMaster.h>

// /* ---------------- RS485 Pins ---------------- */
// #define RS485_DE 4     // Driver Enable
// #define RS485_RE 3     // Receiver Enable (Active LOW)

// /* -------------- Modbus Object --------------- */
// ModbusMaster node;

// /* -------- Direction Control Functions -------- */
// void preTransmission() {
//   digitalWrite(RS485_RE, HIGH);  // Disable receiver
//   digitalWrite(RS485_DE, HIGH);  // Enable transmitter
// }

// void postTransmission() {
//   digitalWrite(RS485_DE, LOW);   // Disable transmitter
//   digitalWrite(RS485_RE, LOW);   // Enable receiver
// }

// void setup() {
//   Serial.begin(115200);
//   while (!Serial) {}

//   /* Serial4: RX=16 TX=17 */
//   Serial4.begin(9600);

//   pinMode(RS485_DE, OUTPUT);
//   pinMode(RS485_RE, OUTPUT);

//   digitalWrite(RS485_DE, LOW);  // TX OFF
//   digitalWrite(RS485_RE, LOW);  // RX ON

//   node.begin(1, Serial4);      // Slave ID = 1
//   node.preTransmission(preTransmission);
//   node.postTransmission(postTransmission);

//   Serial.println("JXBS-3001 NPK Sensor Initialized");
//   Serial.println("Reading registers 0x001E - 0x0020");
// }

// void loop() {
//   uint8_t result;

// //   for (uint16_t addr = 0x001A; addr <= 0x0030; addr++) {
// //     uint8_t res = node.readHoldingRegisters(addr, 1);

// //     if (res == node.ku8MBSuccess) {
// //         Serial.print("Addr 0x");
// //         Serial.print(addr, HEX);
// //         Serial.print(" = ");
// //         Serial.println(node.getResponseBuffer(0));
// //     }
// //     delay(300);
// //     }


//   /* Read N, P, K */
//   result = node.readHoldingRegisters(0x0021, 3);

//   if (result == node.ku8MBSuccess) {
//     uint16_t nitrogen   = node.getResponseBuffer(0); // 0x0021
//     uint16_t phosphorus = node.getResponseBuffer(1); // 0x0022
//     uint16_t potassium  = node.getResponseBuffer(2); // 0x0023

//     Serial.println("-----------------------------");
//     Serial.print("Nitrogen   (mg/kg): ");
//     Serial.println(nitrogen);

//     Serial.print("Phosphorus (mg/kg): ");
//     Serial.println(phosphorus);

//     Serial.print("Potassium  (mg/kg): ");
//     Serial.println(potassium);
//   }
//   else {
//     Serial.print("Modbus Error: ");
//     Serial.println(result);
//   }

//   delay(2000);
// }
