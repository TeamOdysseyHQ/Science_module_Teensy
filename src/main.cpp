#include <Arduino.h>

// together using I2c SCL - 19, SDA- 18
#include "MPU6050Sensor.h"
#include "VL53L0X.h"
#include "BMP280.h"
#include "GasSensors.h"
#include "PH.h"
#include "NPK_ModBuster.h"
#include "TCS3200.h"
#include "DHTSensor.h"

// actuators
#include "LinearActuator.h"
#include "DrillMotor.h"

#define MQ135_ANALOG_PIN A3
#define PH_ANALOG_PIN A0
#define S0 2
#define S1 3
#define S2 4
#define S3 5
#define OUT_PIN 6
#define DHT_PIN 12

// ================== A4988 PIN DEFINITIONS ==================
#define STEP_PIN 7
#define DIR_PIN 8
#define MS1_PIN 8
#define MS2_PIN 9
#define MS3_PIN 10
#define ENABLE_PIN 11

#define MOTOR_PWM  2
#define MOTOR_DIR  5
#define MOTOR_SLP  3



MPU6050_Sensor mpu_sensor;
VL53L0X_Sensor vl53l0x;
BMP280_Sensor bmp_sensor;
// MQ2Sensor mq2(A4);
// MQ4Sensor mq4(A4);
// MQ6Sensor mq6(A2);
MQ135Sensor mq135(MQ135_ANALOG_PIN);
PHSensor ph_sensor(PH_ANALOG_PIN);
NPK_MB_Sensor npk_sensor; // pins to be set in header file
TCS3200_Sensor tcs3200_sensor(S0, S1, S2, S3, OUT_PIN);
DHTSensor dht_sensor(DHT_PIN, DHT22_TYPE); // DHT sensor on pin 12

LinearActuator linear_actuator(STEP_PIN, DIR_PIN, MS1_PIN, MS2_PIN, MS3_PIN, ENABLE_PIN);
DrillMotor drill_motor(MOTOR_PWM, MOTOR_DIR, MOTOR_SLP);

uint16_t distance;
int16_t ax, ay, az;
int16_t gx, gy, gz;
float pH, humidity;
uint16_t nitrogen;
uint16_t phosphorus;
uint16_t potassium;

#ifdef DEBUG_MODE
// Wrapper functions to centralize Serial.print / Serial.println usage
inline void print(const char *s) { Serial.print(s); }
inline void println(const char *s) { Serial.println(s); }

inline void print(const String &s) { Serial.print(s); }
inline void println(const String &s) { Serial.println(s); }

inline void print(const __FlashStringHelper *s) { Serial.print(s); }
inline void println(const __FlashStringHelper *s) { Serial.println(s); }

inline void print(char c) { Serial.print(c); }
inline void println(char c) { Serial.println(c); }

inline void print(bool b) { Serial.print(b); }
inline void println(bool b) { Serial.println(b); }

inline void print(int v) { Serial.print(v); }
inline void println(int v) { Serial.println(v); }

inline void print(unsigned int v) { Serial.print(v); }
inline void println(unsigned int v) { Serial.println(v); }

inline void print(long v) { Serial.print(v); }
inline void println(long v) { Serial.println(v); }

inline void print(unsigned long v) { Serial.print(v); }
inline void println(unsigned long v) { Serial.println(v); }

inline void print(int16_t v) { Serial.print(v); }
inline void println(int16_t v) { Serial.println(v); }

inline void print(uint16_t v) { Serial.print(v); }
inline void println(uint16_t v) { Serial.println(v); }

// inline void print(int32_t v) { Serial.print(v); }
// inline void println(int32_t v) { Serial.println(v); }

// inline void print(uint32_t v) { Serial.print(v); }
// inline void println(uint32_t v) { Serial.println(v); }

inline void print(float v) { Serial.print(v); }
inline void println(float v) { Serial.println(v); }

inline void print(float v, int digits) { Serial.print(v, digits); }
inline void println(float v, int digits) { Serial.println(v, digits); }

// Generic numeric/base overloads (for integer base or float precision)
template<typename T>
inline void print(T v, int base) { Serial.print(v, base); }

template<typename T>
inline void println(T v, int base) { Serial.println(v, base); }
#else
// Wrapper functions to not print in production mode
inline void print(const char *s) { }
inline void println(const char *s) { }

inline void print(const String &s) { }
inline void println(const String &s) { }

inline void print(const __FlashStringHelper *s) { }
inline void println(const __FlashStringHelper *s) { }

inline void print(char c) { }
inline void println(char c) { }

inline void print(bool b) { }
inline void println(bool b) { }

inline void print(int v) { }
inline void println(int v) { }

inline void print(unsigned int v) { }
inline void println(unsigned int v) { }

inline void print(long v) { }
inline void println(long v) { }

inline void print(unsigned long v) { }
inline void println(unsigned long v) { }

inline void print(int16_t v) { }
inline void println(int16_t v) { }

inline void print(uint16_t v) { }
inline void println(uint16_t v) { }

// inline void print(int32_t v) { }
// inline void println(int32_t v) { }

// inline void print(uint32_t v) { }
// inline void println(uint32_t v) { }

inline void print(float v) { }
inline void println(float v) { }

inline void print(float v, int digits) { }
inline void println(float v, int digits) { }

// Generic numeric/base overloads (for integer base or float precision)
template<typename T>
inline void print(T v, int base) { }

template<typename T>
inline void println(T v, int base) { }
#endif

void setup() {
  Serial.begin(115200);

#ifdef DEBUG_MODE
  while (!Serial) {
    ; // Wait for serial monitor to connect
  }
#endif
  // actuators
  linear_actuator.setup();
  drill_motor.setup();
#ifndef ACTUATOR_ONLY_MODE
  //sensors
  mpu_sensor.begin();
  vl53l0x.begin();
  bmp_sensor.begin();
  dht_sensor.begin();
  analogReadResolution(10);

  ph_sensor.begin();
  npk_sensor.begin();
  tcs3200_sensor.begin();
//   mq2.begin();
//   mq4.begin();
//   mq6.begin();
  mq135.begin();

  println("Calibrating sensors in clean air... for 20 seconds");
  delay(20000); // Preheat

//   mq2.calibrate();
//   mq4.calibrate();
//   mq6.calibrate();
    mq135.calibrate();

  println("Calibration complete");
#endif

#ifdef ACTUATOR_ONLY_MODE
  println("=== A4988 Keyboard Control Ready ===");
  println("↑ : Clockwise");
  println("↓ : Anti-clockwise");
  println("0: Full | 1: 1/8 | 2: 1/16 | 3: 1/32");
#endif
}

void loop() {
#ifdef SENSOR_ONLY_MODE
  mpu_sensor.getAcceleration(&ax, &ay, &az);
  mpu_sensor.getRotation(&gx, &gy, &gz);

  println("---------------------------");
  print("Acc: ");
  print(ax); Serial.print(", ");
  print(ay); Serial.print(", ");
  println(az);

  print("Gyro: ");
  print(gx); Serial.print(", ");
  print(gy); Serial.print(", ");
  println(gz);

  distance = vl53l0x.readDistance();
  if (distance != 0xFFFF) {
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" mm");
  } else {
    Serial.println("Out of range");
  }

  print("Temperature: ");
  print(bmp_sensor.readTemperature());
  println(" °C");

  humidity = dht_sensor.getHumidity();
  if (humidity == -999.0f) {
        Serial.println("❌ DHT read error");
  } else {
      Serial.print("Humidity: ");
      Serial.print(humidity);
      Serial.println(" %");
  }

  print("Pressure: ");
  print(bmp_sensor.readPressure());
  println(" hPa");

  print("Approx Altitude: ");
  print(bmp_sensor.readAltitude());
  println(" m");

//   print("MQ-2 Propane: "); print(mq2.readPropane()); println(" ppm");
//   print("MQ-2 CO: "); print(mq2.readCO()); println(" ppm");
//   print("MQ-2 Smoke: "); print(mq2.readSmoke()); println(" ppm");

//   print("MQ-4 Methane: "); print(mq4.readMethane()); println(" ppm");

//   print("MQ-6 Propane: ");.print(mq6.readPropane()); println(" ppm");
//   print("MQ-6 Butane: "); print(mq6.readButane()); println(" ppm");

  print("MQ-135 CO2: "); print(mq135.readCO2()); println(" ppm");
  pH = ph_sensor.readPH();
  print("pH Value: ");
  println(pH, 2);

  if(tcs3200_sensor.isColorViolet()) {
    println("TCS3200: Violet Color Detected");
    } else {
    println("TCS3200: Color Not Violet");
    }

    if (npk_sensor.readNPK(nitrogen, phosphorus, potassium)) {
        print("Nitrogen (mg/kg): ");
        println(nitrogen);
        print("Phosphorus (mg/kg): ");
        println(phosphorus);
        print("Potassium (mg/kg): ");
        println(potassium);
    } else {
        println("Failed to read NPK values from sensor");
    }

  println("---------------------------");

  delay(1000);
#elif defined(ACTUATOR_ONLY_MODE)
  static uint8_t escState = 0;
  if (Serial.available()) {
    char c = Serial.read();

    // --------- Arrow Key Handling (ESC sequences) ---------
    if (escState == 0 && c == 27) { // ESC
      escState = 1;
      return;
    }

    if (escState == 1 && c == '[') {
      escState = 2;
      return;
    }

    if (escState == 2) {
      escState = 0;

      //linear actuator controls
      if (c == 'A') { // Up Arrow
        Serial.println("Rotate CW");
        linear_actuator.rotateMotor(true, linear_actuator.stepsPerMove);
      }
      else if (c == 'B') { // Down Arrow
        Serial.println("Rotate CCW");
        linear_actuator.rotateMotor(false, linear_actuator.stepsPerMove);
      }

      // drill motor controls
      switch (c) {
        case 'q': // CLOCKWISE
          drill_motor.changeDirection(CLOCKWISE);
          Serial.println("Clockwise");
          break;

        case 'e': // ANTICLOCKWISE
          drill_motor.changeDirection(ANTICLOCKWISE);
          Serial.println("Anti-clockwise");
          break;

        case 'C': // Right
          drill_motor.increaseSpeed();
          Serial.print("Speed: ");
          Serial.println(drill_motor.targetSpeed);
          break;

        case 'D': // left
          drill_motor.decreaseSpeed();
          Serial.print("Speed: ");
          Serial.println(drill_motor.targetSpeed);
          break;
      }
      return;
    }

    // --------- Microstep Selection ---------
    switch (c) {
      case '0':
        linear_actuator.currentMicrostep = FULL;
        break;
      case '1':
        linear_actuator.currentMicrostep = MICRO_1_8;
        break;
      case '2':
        linear_actuator.currentMicrostep = MICRO_1_16;
        break;
      case '3':
        linear_actuator.currentMicrostep = MICRO_1_32;
        break;
      default:
        return;
    }

    linear_actuator.setMicrostepping(linear_actuator.currentMicrostep);
  }
#else
// to implement combined mode
#endif
}

