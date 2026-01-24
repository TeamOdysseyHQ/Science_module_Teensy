#ifndef CURRENT_SENSOR_H
#define CURRENT_SENSOR_H

#include <ACS712.h>
#include <Arduino.h>

// ACS712-20A
#define SENSITIVITY 100   // mV/A
#define VREF 3.3
#define ADC_MAX 4095.0

class CurrentSensor
{
public:
    CurrentSensor(int pin);
    void begin();
    float readCurrent();
    void calibrateZeroCurrent();
private:
    int ACS_PIN;
    ACS712 currentSensor;
};

#endif // CURRENT_SENSOR_H
