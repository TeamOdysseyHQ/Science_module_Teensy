#ifndef MQ135_DIGITAL_H
#define MQ135_DIGITAL_H

#include <Arduino.h>

class MQ135_Digital {
private:
    uint8_t _doPin;
    bool _activeLow;   // true if LOW means gas detected, must check and see

public:
    // Constructor
    MQ135_Digital(uint8_t doPin, bool activeLow = true)
        : _doPin(doPin), _activeLow(activeLow) {}

    // Initialize pin
    void begin() {
        pinMode(_doPin, INPUT);
    }

    // Returns true if CO2 is detected
    bool isCO2Detected() {
        int state = digitalRead(_doPin);

        if (_activeLow) {
            return (state == LOW);
        } else {
            return (state == HIGH);
        }
    }

    // Raw digital output (for debugging)
    int readRaw() {
        return digitalRead(_doPin);
    }
};

#endif // MQ135_DIGITAL_H
