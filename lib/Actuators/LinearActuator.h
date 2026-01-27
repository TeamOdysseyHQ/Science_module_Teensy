// #ifndef LINEAR_ACTUATOR_H
// #define LINEAR_ACTUATOR_H


// enum MicrostepMode {
//     FULL = 0,
//     MICRO_1_4 = 4,
//     MICRO_1_8 = 8,
//     MICRO_1_16 = 16
// };

// class LinearActuator
// {
// public:
// // ================== MICROSTEP MODES ==================
//     const int stepsPerMove = 100;   // n steps per arrow press
//     MicrostepMode currentMicrostep = FULL;
//     LinearActuator(int stepPin, int dirPin, int ms1Pin, int ms2Pin, int ms3Pin, int enablePin);
//     void setMicrostepping(MicrostepMode mode);
//     void rotateMotor(bool clockwise, int steps);
//     void setup();
// private:
//     // ================== MOTOR PARAMETERS ==================
//     const int stepDelayMicros = 800; // speed control
//     int STEP_PIN;
//     int DIR_PIN;
//     int MS1_PIN;
//     int MS2_PIN;
//     int MS3_PIN;
//     int ENABLE_PIN;
// };



// #endif 

#ifndef LINEAR_ACTUATOR_H
#define LINEAR_ACTUATOR_H

#include <AccelStepper.h>

class LinearActuator
{
    public:
    /* ================= STATE ================= */
        bool motorRunning = false;
        LinearActuator(int step_pin, int dir_pin, int en_pin, bool isTB6600);
        void moveMotor(bool anticlockwise);
        void setup();
        void run();
    private:
        int STEP_PIN, DIR_PIN, EN_PIN;
        bool isTB6600;
        /* ================= STEPPER ================= */
        AccelStepper stepper = AccelStepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

        /* ================= USER SETTINGS ================= */

        // 🔁 Revolutions per command
        float n_revolutions = 2.0;

        // 🚀 Speed & acceleration
        float motorSpeed = 2000.0;   // steps/sec
        float motorAccel = 2000.0;

        // ⚙️ NEMA 17 motor
        const int fullStepsPerRev = 200;

        /*
        TB6600 MICROSTEP MULTIPLIER (MATCH DIP SWITCH)

        Full step  -> 1
        1/2 step   -> 2
        1/4 step   -> 4
        1/8 step   -> 8
        1/16 step  -> 16
        1/32 step  -> 32
        */
        const int microstepMultiplier = 16;   // <<< SET THIS ONLY
};

#endif