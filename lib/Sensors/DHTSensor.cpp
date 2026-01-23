#include "DHTSensor.h"
#include "CustomPrint.h"

DHTSensor::DHTSensor(uint8_t pin, uint8_t type)
    : dht(pin, type) {}

void DHTSensor::begin() {
    dht.begin();
    println("DHT sensor initialized");
}

float DHTSensor::getTemperature() {
    float temp = dht.readTemperature(); // Celsius
    if (isnan(temp)) return -999.0f;     // error code
    return temp;
}

float DHTSensor::getHumidity() {
    float hum = dht.readHumidity();
    if (isnan(hum)) return -999.0f;      // error code
    return hum;
}
