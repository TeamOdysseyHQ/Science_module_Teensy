#ifndef TEST_TUBE_DROPPER_H
#define TEST_TUBE_DROPPER_H

#include <Arduino.h>
#include <Servo.h>

class TestTubeDropper {
public:
    TestTubeDropper(int servoPin);
    void setup();
    void toggle();
private:
    void moveSmooth(int from, int to);
    int servoPin;
    int angleA = 0;     // start position
    int angleB = 90;    // toggle position
    bool toggled = false;
    Servo servo;
};

#endif // TEST_TUBE_DROPPER_H