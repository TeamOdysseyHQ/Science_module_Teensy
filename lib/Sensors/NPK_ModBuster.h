#ifndef NPK_MODBUSTER_H
#define NPK_MODBUSTER_H

#include <ModbusMaster.h>
  
  /* ---------------- RS485 Pins ---------------- */
#define RS485_DE 5     // Driver Enable
#define RS485_RE 6     // Receiver Enable (Active LOW)
#define RS485Serial Serial4  // Use hardware Serial4 (RX=RO=16, TX=DI=17)

/* -------------- Modbus Object --------------- */




class NPK_MB_Sensor {
public:
    NPK_MB_Sensor();
    void begin();
    bool readNPK(uint16_t &nitrogen, uint16_t &phosphorus, uint16_t &potassium);
private:
    uint8_t result;
    ModbusMaster node_ModBuster;
};  
#endif