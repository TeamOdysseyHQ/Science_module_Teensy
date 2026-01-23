#include "GasSensors.h"
#include "CustomPrint.h"

/************** CLEAN AIR RATIOS **************/
#define MQ2_CLEAN_AIR_RATIO  9.83
#define MQ4_CLEAN_AIR_RATIO  4.4
#define MQ6_CLEAN_AIR_RATIO  10.0
#define MQ135_CLEAN_AIR_RATIO  3.6   // Datasheet value

/************** Constructors **************/
MQ2Sensor::MQ2Sensor(uint8_t analogPin)
  : mq2(Board, VOLTAGE_RESOLUTION, ADC_BIT_RESOLUTION, analogPin, "MQ-2") {}

MQ4Sensor::MQ4Sensor(uint8_t analogPin)
  : mq4(Board, VOLTAGE_RESOLUTION, ADC_BIT_RESOLUTION, analogPin, "MQ-4") {}

MQ6Sensor::MQ6Sensor(uint8_t analogPin)
  : mq6(Board, VOLTAGE_RESOLUTION, ADC_BIT_RESOLUTION, analogPin, "MQ-6") {}

MQ135Sensor::MQ135Sensor(uint8_t analogPin)
  : mq135("Teensy 4.1", VOLTAGE_RESOLUTION, ADC_BIT_RESOLUTION, analogPin, "MQ-135") {}


/************** BEGIN **************/
void MQ2Sensor::begin() {
  mq2.setRegressionMethod(1);
  mq2.init();
  println("MQ-2 sensor initialized");
}

void MQ4Sensor::begin() {
  mq4.setRegressionMethod(1);
  mq4.init();
  println("MQ-4 sensor initialized");
}

void MQ6Sensor::begin() {
  mq6.setRegressionMethod(1);
  mq6.init();
  println("MQ-6 sensor initialized");
}

void MQ135Sensor::begin() {
  mq135.setRegressionMethod(1); // Exponential
  mq135.init();
  println("MQ-135 sensor initialized");
}


/************** CALIBRATION **************/ 
void MQ2Sensor::calibrate() {
  float r0 = 0;
  for (int i = 0; i < 10; i++) {
    mq2.update();
    r0 += mq2.calibrate(MQ2_CLEAN_AIR_RATIO);
    delay(1000);
  }
  println("MQ-2 sensor calibrated with R0: " + String(r0 / 10));
  mq2.setR0(r0 / 10);
}

void MQ4Sensor::calibrate() {
  float r0 = 0;
  for (int i = 0; i < 10; i++) {
    mq4.update();
    r0 += mq4.calibrate(MQ4_CLEAN_AIR_RATIO);
    delay(1000);
  }
  println("MQ-4 sensor calibrated with R0: " + String(r0 / 10));
  mq4.setR0(r0 / 10);
}

void MQ6Sensor::calibrate() {
  float r0 = 0;
  for (int i = 0; i < 10; i++) {
    mq6.update();
    r0 += mq6.calibrate(MQ6_CLEAN_AIR_RATIO);
    delay(1000);
  }
  println("MQ-6 sensor calibrated with R0: " + String(r0 / 10));
  mq6.setR0(r0 / 10);
}

void MQ135Sensor::calibrate() {
  float r0 = 0;
  for (int i = 0; i < 10; i++) {
    mq135.update();
    r0 += mq135.calibrate(MQ135_CLEAN_AIR_RATIO);
    delay(1000);
  }
  println("MQ-135 sensor calibrated with R0: " + String(r0 / 10));
  mq135.setR0(r0 / 10);
}


/************** MQ-2 READINGS **************/
float MQ2Sensor::readPropane() {
  mq2.update();
  mq2.setA(574.25); mq2.setB(-2.222);
  return mq2.readSensor();
}

float MQ2Sensor::readCO() {
  mq2.update();
  mq2.setA(36974); mq2.setB(-3.109);
  return mq2.readSensor();
}

float MQ2Sensor::readSmoke() {
  mq2.update();
  mq2.setA(30000000); mq2.setB(-8.308);
  return mq2.readSensor();
}

/************** MQ-4 READINGS **************/
float MQ4Sensor::readMethane() {
  mq4.update();
  mq4.setA(1012.7); mq4.setB(-2.786);
  return mq4.readSensor();
}

/************** MQ-6 READINGS **************/
float MQ6Sensor::readPropane() {
  mq6.update();
  mq6.setA(2127.2); mq6.setB(-2.526);
  return mq6.readSensor();
}

float MQ6Sensor::readButane() {
  mq6.update();
  mq6.setA(658.71); mq6.setB(-2.168);
  return mq6.readSensor();
}

/************** MQ-135 READINGS **************/
float MQ135Sensor::readCO2() {
  mq135.update();

  // CO2 curve from MQUnifiedsensor documentation
  mq135.setA(110.47);
  mq135.setB(-2.862);

  return mq135.readSensor();
}

