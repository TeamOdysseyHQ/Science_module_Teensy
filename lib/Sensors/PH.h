#ifndef PH_H
#define PH_H

#include <Arduino.h>

class PHSensor {
public:
    PHSensor(int phPin);
    void begin();
    float readPH();
private:
    int _phPin;
    float _ph7Voltage = 1.65; // voltage at pH 7
    float _phSlope = -5.7;    // typical slope (~ -5.7 pH/V)
};

#endif