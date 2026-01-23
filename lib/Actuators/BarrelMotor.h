#ifndef BARREL_MOTOR_H
#define BARREL_MOTOR_H
#include <Arduino.h>

class BarrelMotor
{
public:
    BarrelMotor(int stepPin, int dirPin, int ms1Pin, int ms2Pin, int ms3Pin, int enablePin);
    void setMicrostepping(int mode);
    void rotateDegrees(float degrees);
    void setup();
    void loop();
private:
    // ================== MOTOR PARAMETERS ==================
    const int STEPS_PER_REV = 200; // base steps for 1.8° motor
    const int STEP_PIN;
    const int DIR_PIN;
    const int MS1_PIN;
    const int MS2_PIN;
    const int MS3_PIN;
    const int ENABLE_PIN;

    // Microstep mode
    // 0 = Full
    // 1 = 1/4
    // 2 = 1/8
    // 3 = 1/16
    int microstepMode = 1;

    // Derived
    int microstepMultiplier = 1;
};

#endif