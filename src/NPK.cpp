  /*
  * https://www.electroniclinic.com/
  * Soil NPK Sensor with Arduino for measuring Nitrogen, Phosphorus, and Potassium
  * Modified for Teensy 4.1 - using Hardware Serial
  */
  #include <Wire.h>
  
  // Teensy 4.1 Hardware Serial Pins:
  // Serial1: RX=0, TX=1
  // Serial4: RX=16, TX=17
  // Using Serial4 for RS485 communication
  #define RS485Serial Serial4  // Use hardware Serial4
  #define RS485_RX 16  // Connect RS485 RO to pin 16
  #define RS485_TX 17  // Connect RS485 DI to pin 17
  #define RE 3
  #define DE 4
  
  // The following are the Inquiry frames which are send to the NPK sensor
  //for reading the Nitrogen, Phosphorus, and Potassium values
  // We defined three arrays with names nitro_inquiry_frame, phos_inquiry_frame, and pota_inquiry_frame 
  // Each inquiry frame have 8 values
  const byte nitro_inquiry_frame[] = {0x01,0x03, 0x00, 0x1e, 0x00, 0x01, 0xe4, 0x0c}; 
  const byte phos_inquiry_frame[] = {0x01,0x03, 0x00, 0x1f, 0x00, 0x01, 0xb5, 0xcc}; 
  const byte pota_inquiry_frame[] = {0x01,0x03, 0x00, 0x20, 0x00, 0x01, 0x85, 0xc0};
  // const byte nitro_inquiry_frame[] = {0x01,0x03,0x00,0x21,0x00,0x01,0xD4,0x00};
  // const byte phos_inquiry_frame[] = {0x01,0x03,0x00,0x22,0x00,0x01,0x24,0x00};
  // const byte pota_inquiry_frame[] = {0x01,0x03,0x00,0x23,0x00,0x01,0x75,0xC0};

  
  byte values[11];
  // Using hardware Serial4 instead of SoftwareSerial for Teensy 4.1
uint16_t nitrogen();
uint16_t phosphorous();
uint16_t potassium();

  /*
  * Setup function. Here we do the basics
  */
  // void setup(void)
  // {
  //   // start serial port
  //   Serial.begin(115200);

  //   // Try 4800 if 9600 doesn't work (some sensors use 4800)
  //   RS485Serial.begin(9600);  // Hardware serial for Teensy 4.1
  //   pinMode(RE, OUTPUT);
  //   pinMode(DE, OUTPUT);
    
  //   // Set to receive mode initially
  //   digitalWrite(DE, LOW);
  //   digitalWrite(RE, LOW);
    
  //   Serial.println("NPK Sensor Test - Teensy 4.1 - Baud: 9600");
  //   Serial.println("Using Serial4: RX=Pin16, TX=Pin17");
  //   Serial.println("Make sure: 12V GND is connected to Teensy GND!");
  //   delay(3000);
  // }
  // /*
  // * Main function. It will request the tempC from the sensors and display on Serial.
  // */
  // void loop(void)
  // { 
  //   uint16_t nitrogen_val,phosphorus_val,potassium_val;
  
  //   nitrogen_val = nitrogen();
  //   // delay(250);
  //   phosphorus_val = phosphorous();
  //   // delay(250);
  //   potassium_val = potassium();
  //   // delay(250);
    
  //   Serial.print("Nitrogen_Val: ");
  //   Serial.print(nitrogen_val);
  //   Serial.println(" mg/kg");
  //   Serial.print("Phosphorous_Val: ");
  //   Serial.print(phosphorus_val);
  //   Serial.println(" mg/kg");
  //   Serial.print("Potassium_Val: ");
  //   Serial.print(potassium_val);
  //   Serial.println(" mg/kg");
  //   delay(2000);
  // }

  uint16_t nitrogen(){
    // Clear any old data in buffer
    while(RS485Serial.available()) RS485Serial.read();
    
    digitalWrite(DE,HIGH);
    digitalWrite(RE,HIGH);
    RS485Serial.write(nitro_inquiry_frame, sizeof(nitro_inquiry_frame));
    RS485Serial.flush(); // Wait for transmission to complete
    digitalWrite(DE,LOW);
    digitalWrite(RE,LOW);
    
    // Wait for response with timeout
    unsigned long startTime = millis();
    while(RS485Serial.available() < 7 && millis() - startTime < 1000) {
      delay(1);
    }
    
    Serial.print("N bytes received: ");
    Serial.print(RS485Serial.available());
    Serial.print(" -> ");
    
    if(RS485Serial.available() >= 7) {
      for(byte i=0; i<7; i++){
        values[i] = RS485Serial.read();
        Serial.print(values[i], HEX);
        Serial.print(" ");
      }
    } else {
      Serial.print("NO RESPONSE");
    }
    Serial.println();
    return (values[3] << 8) | values[4];
  }
  
  uint16_t phosphorous(){
    while(RS485Serial.available()) RS485Serial.read();
    
    digitalWrite(DE,HIGH);
    digitalWrite(RE,HIGH);
    // delay(10);
    RS485Serial.write(phos_inquiry_frame, sizeof(phos_inquiry_frame));
    RS485Serial.flush();
    // delay(10);
    digitalWrite(DE,LOW);
    digitalWrite(RE,LOW);
    
    unsigned long startTime = millis();
    while(RS485Serial.available() < 7 && millis() - startTime < 1000) {
      delay(1);
    }
    
    Serial.print("P bytes received: ");
    Serial.print(RS485Serial.available());
    Serial.print(" -> ");
    
    if(RS485Serial.available() >= 7) {
      for(byte i=0; i<7; i++){
        values[i] = RS485Serial.read();
        Serial.print(values[i], HEX);
        Serial.print(" ");
      }
    } else {
      Serial.print("NO RESPONSE");
    }
    Serial.println();
    return (values[3] << 8) | values[4];
  }
  
  uint16_t potassium(){
    while(RS485Serial.available()) RS485Serial.read();
    
    digitalWrite(DE,HIGH);
    digitalWrite(RE,HIGH);
    // delay(10);
    RS485Serial.write(pota_inquiry_frame, sizeof(pota_inquiry_frame));
    RS485Serial.flush(); // Wait for transmission to complete
    // delay(10);
    digitalWrite(DE,LOW);
    digitalWrite(RE,LOW);
    
    // Wait for response with timeout
    unsigned long startTime = millis();
    while(RS485Serial.available() < 7 && millis() - startTime < 1000) {
      delay(1);
    }
    
    Serial.print("K bytes received: ");
    Serial.print(RS485Serial.available());
    Serial.print(" -> ");
    
    if(RS485Serial.available() >= 7) {
      for(byte i=0; i<7; i++){
        values[i] = RS485Serial.read();
        Serial.print(values[i], HEX);
        Serial.print(" ");
      }
    } else {
      Serial.print("NO RESPONSE");
    }
    Serial.println();
    return (values[3] << 8) | values[4];
  }
 
