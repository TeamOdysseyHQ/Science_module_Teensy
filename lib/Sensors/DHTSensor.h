#ifndef DHTSENSOR_H
#define DHTSENSOR_H

#include <Arduino.h>
#include <DHT.h>

// Sensor type aliases
#define DHT11_TYPE DHT11
#define DHT22_TYPE DHT22

class DHTSensor {
private:
    DHT dht;

public:
    DHTSensor(uint8_t pin, uint8_t type);
    void begin();

    float getTemperature();   // °C
    float getHumidity();      // %
};

#endif
