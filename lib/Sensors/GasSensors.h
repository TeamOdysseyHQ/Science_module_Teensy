#ifndef GAS_SENSORS_H
#define GAS_SENSORS_H

#include <Arduino.h>
#include <MQUnifiedsensor.h>

/************** Common Hardware Config **************/
#define VOLTAGE_RESOLUTION  3.3
#define ADC_BIT_RESOLUTION  10
#define Board               "Teensy 4.1"

/************** MQ-2 (Propane, Smoke, Alcohol, CO) **************/
class MQ2Sensor {
public:
  MQ2Sensor(uint8_t analogPin);
  void begin();
  void calibrate();
  float readPropane();
  float readCO();
  float readSmoke();

private:
  MQUnifiedsensor mq2;
};

/************** MQ-4 (Methane) **************/
class MQ4Sensor {
public:
  MQ4Sensor(uint8_t analogPin);
  void begin();
  void calibrate();
  float readMethane();

private:
  MQUnifiedsensor mq4;
};

/************** MQ-6 (Butane, Propane) **************/
class MQ6Sensor {
public:
  MQ6Sensor(uint8_t analogPin);
  void begin();
  void calibrate();
  float readPropane();
  float readButane();

private:
  MQUnifiedsensor mq6;
};

/************** MQ-135 (CO2) **************/
class MQ135Sensor {
public:
  MQ135Sensor(uint8_t analogPin);
  void begin();
  void calibrate();
  float readCO2();

private:
  MQUnifiedsensor mq135;
};

#endif
