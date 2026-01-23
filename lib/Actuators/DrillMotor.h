#ifndef DRILL_MOTOR_H
#define DRILL_MOTOR_H

#include <Arduino.h>
enum Direction {
  STOPPED,
  CLOCKWISE,
  ANTICLOCKWISE
};

class DrillMotor {
public:
    int targetSpeed = 120;     // Desired speed (0–255)
    DrillMotor(int pwmPin, int dirPin, int slpPin);
    void setup();
    void changeDirection(Direction newDir);
    void increaseSpeed();
    void decreaseSpeed();
private:
    // when connecting with base center they will give in percentage, must converted to 0-255 for PWM
    int currentSpeed = 0;
    int MOTOR_PWM, MOTOR_DIR, MOTOR_SLP;
    const int speedStep = 10;
    const int rampDelay = 15;     // ms between PWM steps
    const int dirChangeDelay = 1000;
    Direction currentDir = STOPPED;
    Direction requestedDir = STOPPED;
    void handleDirectionChange();
    void rampToSpeed(int newSpeed);
};

#endif