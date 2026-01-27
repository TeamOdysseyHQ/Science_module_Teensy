#ifndef BARREL_MOTOR_H
#define BARREL_MOTOR_H

#include <AccelStepper.h>

class BarrelMotor
{
public:
    BarrelMotor(int step_pin, int dir_pin, int en_pin, bool isTB6600);
    void setup();
    void run();
    void handleBarellRotateInput(bool forward);

private:
    int STEP_PIN, DIR_PIN, ENABLE_PIN;
    bool isTB6600;
    // ================= MOTOR SETUP =====================
    AccelStepper stepper = AccelStepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

    // ================= MOTOR SPECS =====================
    const int BASE_STEPS_PER_REV = 200;   // NEMA 17 (1.8°)
    const int MICROSTEP = 1;             // MUST match TB6600 DIP switches
    const float ROTATE_DEG = 240.0;

    // ================= CALCULATED ======================
    const int STEPS_PER_REV = BASE_STEPS_PER_REV * MICROSTEP;
    const int STEPS_240_DEG  = (STEPS_PER_REV * ROTATE_DEG) / 360.0;

    // ================= SPEED CONTROL ===================
    float stepperSpeed = 1000.0;           
    float stepperAccel = 1000.0;

    // ================= STATE ===========================
    bool motorRunning = false;
    void rotate240deg(bool forward);
};

#endif