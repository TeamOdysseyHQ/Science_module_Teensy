#include <Arduino.h>
#include "CustomPrint.h"

// together using I2c SCL - 19, SDA- 18
#include "MPU6050Sensor.h"
#include "VL53L0X.h"
#include "BMP280.h"
#include "GasSensors.h"
// #include "PH.h"
#include "NPK_ModBuster.h"
#include "TCS3200.h"
#include "DHTSensor.h"
#include "CurrentSensor.h"

// actuators
#include "LinearActuator.h"
#include "DrillMotor.h"
#include "BarrelMotor.h"
#include "PHServo.h"

#define DELAY_BETWEEN_SENSOR_READS 1000 // loop steps

#define MQ135_ANALOG_PIN A3
// #define PH_ANALOG_PIN A0
#define ACS_PIN A0
#define S0 2
#define S1 3
#define S2 4
#define S3 5
#define OUT_PIN 6
#define DHT_PIN 12

// ================== Linear Actuator PIN DEFINITIONS ==================
#define LA_STEP_PIN 7
#define LA_DIR_PIN 8
#define LA_MS1_PIN 8
#define LA_MS2_PIN 9
#define LA_MS3_PIN 10
#define LA_ENABLE_PIN 11

// --------------------------------- Drill Motor PIN DEFINITIONS -------------------------------
#define MOTOR_PWM  2
#define MOTOR_DIR  5
#define MOTOR_SLP  3

// ================= Barrel pin definitions ==================
#define B_STEP_PIN 7    
#define B_DIR_PIN 8
#define B_MS1_PIN 8
#define B_MS2_PIN 9
#define B_MS3_PIN 10
#define B_ENABLE_PIN 11
#define BAREL_ROTATE_DEG 60.0

#define SERVO_PIN 9 


MPU6050_Sensor mpu_sensor;
VL53L0X_Sensor vl53l0x;
BMP280_Sensor bmp_sensor;
// MQ2Sensor mq2(A4);
// MQ4Sensor mq4(A4);
// MQ6Sensor mq6(A2);
MQ135Sensor mq135(MQ135_ANALOG_PIN);
// PHSensor ph_sensor(PH_ANALOG_PIN);
NPK_MB_Sensor npk_sensor; // pins to be set in header file
TCS3200_Sensor tcs3200_sensor(S0, S1, S2, S3, OUT_PIN);
DHTSensor dht_sensor(DHT_PIN, DHT22_TYPE); // DHT sensor on pin 12
CurrentSensor current_sensor(ACS_PIN);

LinearActuator linear_actuator(LA_STEP_PIN, LA_DIR_PIN, LA_MS1_PIN, LA_MS2_PIN, LA_MS3_PIN, LA_ENABLE_PIN);
DrillMotor drill_motor(MOTOR_PWM, MOTOR_DIR, MOTOR_SLP);
BarrelMotor barrel_motor(B_STEP_PIN, B_DIR_PIN, B_MS1_PIN, B_MS2_PIN, B_MS3_PIN, B_ENABLE_PIN);
PHServo ph_servo(SERVO_PIN);

uint16_t distance, initial_distance_just_after_starting_drill = 0, dist_bw_sensor_drill = 5, cm_to_drill = 10;
int16_t ax, ay, az;
int16_t gx, gy, gz;
// thresholds for shaking detection
int16_t ax_th_x_min = -1000, ay_th_y_min = -1000, az_th_z_min = -1000;
int16_t ax_th_x_max = 1000, ay_th_y_max = 1000, az_th_z_max = 1000;
int16_t gx_th_x_min = -1000, gy_th_y_min = -1000, gz_th_z_min = -1000;
int16_t gx_th_x_max = 1000, gy_th_y_max = 1000, gz_th_z_max = 1000;
float pH, humidity, drill_motor_ma, drill_motor_ma_threshold = 500.0f;
uint16_t nitrogen;
uint16_t phosphorus;
uint16_t potassium;

bool scienceModeEnable = false;
bool isBFirstPressed = false;
bool isStartDrillFlagged = false;
bool isStopDrillFlagged = false;
bool isShakeFlagged = false;
bool isCurrentThresholdExceededFlagged = false;

int delayCounter = 0;

void setup() {
  Serial.begin(115200);

#ifdef DEBUG_MODE
  while (!Serial) {
    ; // Wait for serial monitor to connect
  }
#endif

#ifndef SENSOR_ONLY_MODE // cause this sensor only works with actuators
  // current sensor
  current_sensor.begin();
  delay(2000); // allow sensor to stabilize
  current_sensor.calibrateZeroCurrent();
#endif

  // actuators
  linear_actuator.setup();
  drill_motor.setup();
  barrel_motor.setup();
  ph_servo.setup();
#ifndef ACTUATOR_ONLY_MODE
  //sensors
  mpu_sensor.begin();
  vl53l0x.begin();
  bmp_sensor.begin();
  dht_sensor.begin();
  analogReadResolution(10);

  // ph_sensor.begin();
  npk_sensor.begin();
  tcs3200_sensor.begin();
//   mq2.begin();
//   mq4.begin();
//   mq6.begin();
  mq135.begin();

  println("Preheating MQ sensors for 20 seconds");
  delay(20000); // Preheat

//   mq2.calibrate();
//   mq4.calibrate();
//   mq6.calibrate();
  println("MQ sensor preheat complete, calibrating sensors...");
  mq135.calibrate();
  println("Calibration complete");
#endif

#ifdef ACTUATOR_ONLY_MODE
  println("=== Keyboard Control Ready ===");
  println("↑ : Clockwise");
  println("↓ : Anti-clockwise");
  println("← : Drill Motor decrease speed");
  println("→ : Drill Motor increase speed");
  println("q: Drill Motor Clockwise");
  println("e: Drill Motor Anti-clockwise");
  println("b: Barrel Motor Rotate");
  println("s: Toggle PH Servo Position");
  println("0: Full | 1: 1/4 | 2: 1/8 | 3: 1/16");
#elif defined(DEBUG_MODE)
  println("=== Keyboard Control Ready ===");
  println("y: Toggle Science exploration mode");
  println("↑ : Clockwise");
  println("↓ : Anti-clockwise");
  println("← : Drill Motor decrease speed");
  println("→ : Drill Motor increase speed");
  println("q: Drill Motor Clockwise");
  println("e: Drill Motor Anti-clockwise");
  println("b: Barrel Motor Rotate");
  println("s: Toggle PH Servo Position");
  println("0: Full | 1: 1/4 | 2: 1/8 | 3: 1/16");
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

  // pH = ph_sensor.readPH();
  // print("pH Value: ");
  // println(pH, 2);

  if(tcs3200_sensor.isColorViolet()) {
    println("TCS3200: Violet Color Detected");
    } else {
    println("TCS3200: Color Not Violet");
    }
  if(tcs3200_sensor.isColorless()) {
      println("TCS3200: Colorless Detected");
      } 
    else {
      println("TCS3200: Color Not Colorless");
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
  print("Current (mA): ");
  println(current_sensor.readCurrent());
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

      //All controls
      switch (c) {
        // Linear Actuator controls
        case 'A': // Up Arrow
          Serial.println("LA Rotate CW");
          linear_actuator.rotateMotor(true, linear_actuator.stepsPerMove);
          break;
        case 'B': // Down Arrow
          Serial.println("LA Rotate CCW");
          linear_actuator.rotateMotor(false, linear_actuator.stepsPerMove);
          break;
        // drill motor controls
        case 'q': // CLOCKWISE
          drill_motor.changeDirection(CLOCKWISE);
          Serial.println("drill Clockwise");
          break;

        case 'e': // ANTICLOCKWISE
          drill_motor.changeDirection(ANTICLOCKWISE);
          Serial.println("drill Anti-clockwise");
          break;

        case 'C': // Right
          drill_motor.increaseSpeed();
          Serial.print("drill Speed: ");
          Serial.println(drill_motor.targetSpeed);
          break;

        case 'D': // left
          drill_motor.decreaseSpeed();
          Serial.print("drill Speed: ");
          Serial.println(drill_motor.targetSpeed);
          break;
        case 'b': // Barrel Motor Rotate
          Serial.println("Barrel Motor Rotate 60 degrees");
          barrel_motor.rotateDegrees(BAREL_ROTATE_DEG);
          break;
        // PH Servo Toggle
        case 's':
        case 'S':
          Serial.println("Toggling PH Servo Position");
          ph_servo.togglePosition();
          break;
        // --------- Microstep Selection ---------
        case '0':
          linear_actuator.setMicrostepping(FULL);
          break;
        case '1':
          linear_actuator.setMicrostepping(MICRO_1_4);
          break;
        case '2':
          linear_actuator.setMicrostepping(MICRO_1_8);
          break;
        case '3':
          linear_actuator.setMicrostepping(MICRO_1_16);
          break;
      }
    }
  }
#elif defined(DEBUG_MODE)
  if (Serial.available()) {
    char c = Serial.read();

    if (c == 'y') {
      scienceModeEnable = !scienceModeEnable;
      if (scienceModeEnable) {
        println("Science Exploration Mode Enabled");
      } else {
        println("Science Exploration Mode Disabled");
        isBFirstPressed = false;
        isStartDrillFlagged = false;
        isStopDrillFlagged = false;
        isShakeFlagged = false;
        isCurrentThresholdExceededFlagged = false;
        initial_distance_just_after_starting_drill = 0;
      }
      return;
    }

    if (!scienceModeEnable) {
      return; // Ignore other inputs if science mode is not enabled
    }
    //All controls
    switch (c) {
      // Linear Actuator controls
      case 'A': // Up Arrow
        Serial.println("LA Rotate CW");
        linear_actuator.rotateMotor(true, linear_actuator.stepsPerMove);
        break;
      case 'B': // Down Arrow
        Serial.println("LA Rotate CCW");
        linear_actuator.rotateMotor(false, linear_actuator.stepsPerMove);
        break;
      // drill motor controls
      case 'Q':
      case 'q': // CLOCKWISE
        drill_motor.changeDirection(CLOCKWISE);
        Serial.println("drill Clockwise");
        break;
      case 'E':
      case 'e': // ANTICLOCKWISE
        drill_motor.changeDirection(ANTICLOCKWISE);
        Serial.println("drill Anti-clockwise");
        break;

      case 'C': // Right
        drill_motor.increaseSpeed();
        Serial.print("drill Speed: ");
        Serial.println(drill_motor.targetSpeed);
        break;

      case 'D': // left
        drill_motor.decreaseSpeed();
        Serial.print("drill Speed: ");
        Serial.println(drill_motor.targetSpeed);
        break;
      case 'b': // Barrel Motor Rotate
        Serial.println("Barrel Motor Rotate 60 degrees");
        isBFirstPressed = true;
        barrel_motor.rotateDegrees(BAREL_ROTATE_DEG);
        break;
      // PH Servo Toggle
        case 's':
        case 'S':
          if (isBFirstPressed)
          {
            Serial.println("Toggling PH Servo Position");
            ph_servo.togglePosition();
          }
          break;
      // --------- Microstep Selection ---------
      case '0':
        linear_actuator.setMicrostepping(FULL);
        barrel_motor.setMicrostepping(FULL);
        break;
      case '1':
        linear_actuator.setMicrostepping(MICRO_1_4);
        barrel_motor.setMicrostepping(MICRO_1_4);
        break;
      case '2':
        linear_actuator.setMicrostepping(MICRO_1_8);
        barrel_motor.setMicrostepping(MICRO_1_8);
        break;
      case '3':
        linear_actuator.setMicrostepping(MICRO_1_16);
        barrel_motor.setMicrostepping(MICRO_1_16);
        break;
    }
    return;
  }

  delayCounter += 1;

  // print drill values only after every defined delay
  if(delayCounter > DELAY_BETWEEN_SENSOR_READS)
  {
    // get drill sensor values
    mpu_sensor.getAcceleration(&ax, &ay, &az);
    mpu_sensor.getRotation(&gx, &gy, &gz);
    distance = vl53l0x.readDistance();
    drill_motor_ma = current_sensor.readCurrent();

    if(distance - dist_bw_sensor_drill <=0 && !isStartDrillFlagged) {
      println("INFO: start drilling");
      initial_distance_just_after_starting_drill = distance;
      isStartDrillFlagged = true;
    }

    if(initial_distance_just_after_starting_drill - distance >= cm_to_drill*10 && isStartDrillFlagged && !isStopDrillFlagged) {
      println("WARNING: stop drilling");
      isStopDrillFlagged = true;
    }

    if((ax < ax_th_x_min || ax > ax_th_x_max ||
        ay < ay_th_y_min || ay > ay_th_y_max ||
        az < az_th_z_min || az > az_th_z_max ||
        gx < gx_th_x_min || gx > gx_th_x_max ||
        gy < gy_th_y_min || gy > gy_th_y_max ||
        gz < gz_th_z_min || gz > gz_th_z_max) && !isShakeFlagged) {
      println("ALERT: Shake Detected!");
      isShakeFlagged = true;
    }

    if(drill_motor_ma > drill_motor_ma_threshold && !isCurrentThresholdExceededFlagged) {
      println("ALERT: Current Threshold Exceeded!");
      isCurrentThresholdExceededFlagged = true;
    }

    println("---------------------------");
    print("Acc: ");
    print(ax); Serial.print(", ");
    print(ay); Serial.print(", ");
    println(az);

    print("Gyro: ");
    print(gx); Serial.print(", ");
    print(gy); Serial.print(", ");
    println(gz);


    if (distance != 0xFFFF) {
      Serial.print("Distance: ");
      Serial.print(distance);
      Serial.println(" mm");
    } else {
      Serial.println("Out of range");
    }
  }

  if (isBFirstPressed && delayCounter > DELAY_BETWEEN_SENSOR_READS) {
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

    // pH = ph_sensor.readPH();
    // print("pH Value: ");
    // println(pH, 2);

    if(tcs3200_sensor.isColorViolet()) {
      println("TCS3200: Violet Color Detected");
      } 
    else {
      println("TCS3200: Color Not Violet");
    }
    if(tcs3200_sensor.isColorless()) {
      println("TCS3200: Colorless Detected");
      } 
    else {
      println("TCS3200: Color Not Colorless");
    }

    if (npk_sensor.readNPK(nitrogen, phosphorus, potassium)) {
      print("Nitrogen (mg/kg): ");
      println(nitrogen);
      print("Phosphorus (mg/kg): ");
      println(phosphorus);
      print("Potassium (mg/kg): ");
      println(potassium);
    } 
    else 
    {
      println("Failed to read NPK values from sensor");
    }

    println("---------------------------");
  }

  if (delayCounter > DELAY_BETWEEN_SENSOR_READS) {
    delayCounter = 0;
  }
#else // Production code
//Note: to be implemented combined mode
#endif
}

