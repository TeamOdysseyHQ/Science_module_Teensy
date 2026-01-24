#ifndef PHSERVO_H
#define PHSERVO_H

#include <Servo.h>

class PHServo
{
public:
    PHServo(int servoPin);
    void setup();
    void togglePosition();
private:
    int SERVO_PIN;
    const int CENTER_ANGLE = 90; // Neutral position
    Servo myServo;
    bool direction = false;   // false = -90, true = +90
};

#endif // PHSERVO_H