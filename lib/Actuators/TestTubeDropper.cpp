#include "TestTubeDropper.h"
#include "CustomPrint.h"

TestTubeDropper::TestTubeDropper(int servoPin) : servoPin(servoPin) {}

void TestTubeDropper::setup() {
  servo.attach(servoPin);
  servo.write(angleA);  // initial position
  delay(500);
  println("Press 's' to toggle servo 0° <-> 90°");
}

void TestTubeDropper::toggle() {
  toggled = !toggled;

  if (toggled) {
    moveSmooth(angleA, angleB);
    println("Servo -> 90°");
  } else {
    moveSmooth(angleB, angleA);
    println("Servo -> 0°");
  }
}

/* Smooth movement to protect servo */
void TestTubeDropper::moveSmooth(int from, int to) {
  if (from < to) {
    for (int pos = from; pos <= to; pos++) {
      servo.write(pos);
      // delay(10);
    }
  } else {
    for (int pos = from; pos >= to; pos--) {
      servo.write(pos);
      // delay(10);
    }
  }
}