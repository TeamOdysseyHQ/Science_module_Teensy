// #include <Arduino.h>
// #include <MQUnifiedsensor.h>

// /************** Hardware Config **************/
// #define VOLTAGE_RESOLUTION   3.3
// #define ADC_BIT_RESOLUTION   10

// /************** MQ-2 **************/
// class MQ2Sensor {
// public:
//   MQ2Sensor(uint8_t pin);
//   void begin();
//   void calibrate();
//   float propane();
//   float co();
//   float smoke();
// private:
//   MQUnifiedsensor mq;
// };

// /************** MQ-4 **************/
// class MQ4Sensor {
// public:
//   MQ4Sensor(uint8_t pin);
//   void begin();
//   void calibrate();
//   float methane();
// private:
//   MQUnifiedsensor mq;
// };

// /************** MQ-6 **************/
// class MQ6Sensor {
// public:
//   MQ6Sensor(uint8_t pin);
//   void begin();
//   void calibrate();
//   float propane();
//   float butane();
// private:
//   MQUnifiedsensor mq;
// };

// /************** MQ-135 **************/
// class MQ135Sensor {
// public:
//   MQ135Sensor(uint8_t pin);
//   void begin();
//   void calibrate();
//   float co2();
// private:
//   MQUnifiedsensor mq;
// };

// /************** Clean Air Ratios **************/
// #define MQ2_RATIO     9.83
// #define MQ4_RATIO     4.4
// #define MQ6_RATIO     10.0
// #define MQ135_RATIO   3.6

// /************** Constructors **************/
// MQ2Sensor::MQ2Sensor(uint8_t pin)
//   : mq("Teensy 4.1", VOLTAGE_RESOLUTION, ADC_BIT_RESOLUTION, pin, "MQ-2") {}

// MQ4Sensor::MQ4Sensor(uint8_t pin)
//   : mq("Teensy 4.1", VOLTAGE_RESOLUTION, ADC_BIT_RESOLUTION, pin, "MQ-4") {}

// MQ6Sensor::MQ6Sensor(uint8_t pin)
//   : mq("Teensy 4.1", VOLTAGE_RESOLUTION, ADC_BIT_RESOLUTION, pin, "MQ-6") {}

// MQ135Sensor::MQ135Sensor(uint8_t pin)
//   : mq("Teensy 4.1", VOLTAGE_RESOLUTION, ADC_BIT_RESOLUTION, pin, "MQ-135") {}

// /************** Begin **************/
// void MQ2Sensor::begin()    { mq.setRegressionMethod(1); mq.init(); }
// void MQ4Sensor::begin()    { mq.setRegressionMethod(1); mq.init(); }
// void MQ6Sensor::begin()    { mq.setRegressionMethod(1); mq.init(); }
// void MQ135Sensor::begin()  { mq.setRegressionMethod(1); mq.init(); }

// /************** Calibration **************/
// void MQ2Sensor::calibrate() {
//   float r0 = 0;
//   for (int i = 0; i < 10; i++) { mq.update(); r0 += mq.calibrate(MQ2_RATIO); delay(1000); }
//   mq.setR0(r0 / 10);
// }

// void MQ4Sensor::calibrate() {
//   float r0 = 0;
//   for (int i = 0; i < 10; i++) { mq.update(); r0 += mq.calibrate(MQ4_RATIO); delay(1000); }
//   mq.setR0(r0 / 10);
// }

// void MQ6Sensor::calibrate() {
//   float r0 = 0;
//   for (int i = 0; i < 10; i++) { mq.update(); r0 += mq.calibrate(MQ6_RATIO); delay(1000); }
//   mq.setR0(r0 / 10);
// }

// void MQ135Sensor::calibrate() {
//   float r0 = 0;
//   for (int i = 0; i < 10; i++) { mq.update(); r0 += mq.calibrate(MQ135_RATIO); delay(1000); }
//   mq.setR0(r0 / 10);
// }

// /************** MQ-2 Readings **************/
// float MQ2Sensor::propane() { mq.update(); mq.setA(574.25); mq.setB(-2.222); return mq.readSensor(); }
// float MQ2Sensor::co()      { mq.update(); mq.setA(36974);  mq.setB(-3.109); return mq.readSensor(); }
// float MQ2Sensor::smoke()   { mq.update(); mq.setA(30000000); mq.setB(-8.308); return mq.readSensor(); }

// /************** MQ-4 Reading **************/
// float MQ4Sensor::methane() { mq.update(); mq.setA(1012.7); mq.setB(-2.786); return mq.readSensor(); }

// /************** MQ-6 Readings **************/
// float MQ6Sensor::propane() { mq.update(); mq.setA(2127.2); mq.setB(-2.526); return mq.readSensor(); }
// float MQ6Sensor::butane()  { mq.update(); mq.setA(658.71); mq.setB(-2.168); return mq.readSensor(); }

// /************** MQ-135 Reading **************/
// float MQ135Sensor::co2()   { mq.update(); mq.setA(110.47); mq.setB(-2.862); return mq.readSensor(); }

// MQ2Sensor mq2(A4);
// MQ4Sensor mq4(A4);
// MQ6Sensor mq6(A2);
// MQ135Sensor mq135(A3);

// void setup()
// {
//     Serial.begin(115200);
//     while (!Serial)
//     {
//         delay(10);
//     }

//     mq2.begin();
//     mq4.begin();
//     mq6.begin();
//     mq135.begin();

//     Serial.println("Calibrating sensors in clean air... for 20 seconds");
//     delay(20000); // Preheat

//     mq2.calibrate();
//     mq4.calibrate();
//     mq6.calibrate();
//     mq135.calibrate();
// }

// void loop()
// {
//     Serial.print("MQ-2 Propane: "); Serial.print(mq2.propane()); Serial.println(" ppm");
//     Serial.print("MQ-2 CO: "); Serial.print(mq2.co()); Serial.println(" ppm");
//     Serial.print("MQ-2 Smoke: "); Serial.print(mq2.smoke()); Serial.println(" ppm");

//     Serial.print("MQ-4 Methane: "); Serial.print(mq4.methane()); Serial.println(" ppm");

//     Serial.print("MQ-6 Propane: "); Serial.print(mq6.propane()); Serial.println(" ppm");
//     Serial.print("MQ-6 Butane: "); Serial.print(mq6.butane()); Serial.println(" ppm");

//     Serial.print("MQ-135 CO2: "); Serial.print(mq135.co2()); Serial.println(" ppm");

//     Serial.println("--------------------");
//     delay(5000);
// }
