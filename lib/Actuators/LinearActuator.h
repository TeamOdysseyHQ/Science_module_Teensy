#ifndef LINEAR_ACTUATOR_H
#define LINEAR_ACTUATOR_H


enum MicrostepMode {
    FULL = 0,
    MICRO_1_4 = 4,
    MICRO_1_8 = 8,
    MICRO_1_16 = 16
};

class LinearActuator
{
public:
// ================== MICROSTEP MODES ==================
    const int stepsPerMove = 100;   // n steps per arrow press
    MicrostepMode currentMicrostep = FULL;
    LinearActuator(int stepPin, int dirPin, int ms1Pin, int ms2Pin, int ms3Pin, int enablePin);
    void setMicrostepping(MicrostepMode mode);
    void rotateMotor(bool clockwise, int steps);
    void setup();
private:
    // ================== MOTOR PARAMETERS ==================
    const int stepDelayMicros = 800; // speed control
    int STEP_PIN;
    int DIR_PIN;
    int MS1_PIN;
    int MS2_PIN;
    int MS3_PIN;
    int ENABLE_PIN;
};



#endif 