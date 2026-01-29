#include <Arduino.h>
#include "CustomPrint.h"

// together using I2c SCL - 19, SDA- 18
#include "MPU6050Sensor.h"
#include "VL53L0X.h"
#include "BMP280.h"
#include "MQ135_digital.h"
// #include "GasSensors.h"
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
#include "TestTubeDropper.h"

#include "PubSub.h"

//Important: Put actual port no., calibirate threshold, calibirate color sensor, calibirate LA step size, calibirate barell step size

// important: TB6600 - enable pin (9) - LOW, B126 - enable pin (12) - HIGH

#define DELAY_BETWEEN_SENSOR_READS 1000 // loop steps

// #define MQ135_ANALOG_PIN A3 // change to digital
#define MQ135_DIGITAL_PIN 5
// #define PH_ANALOG_PIN A0
#define ACS_PIN A14
// #define S0 2
// #define S1 3
// #define S2 4
// #define S3 5
// #define OUT_PIN 6
#define DHT_PIN 6

// ================== Linear Actuator PIN DEFINITIONS (TB6600 motor driver) ==================
#define LA_STEP_PIN 7
#define LA_DIR_PIN 8
#define LA_ENABLE_PIN 9
#define IS_LA_TB6600 true

// --------------------------------- Drill Motor PIN DEFINITIONS -------------------------------
#define MOTOR_PWM  33
#define MOTOR_DIR  34
#define MOTOR_SLP  35

// ================= Barrel pin definitions (B126 motor driver) ==================
#define B_STEP_PIN 10  
#define B_DIR_PIN 11
#define B_ENABLE_PIN 12
#define IS_BARELL_TB6600 false

#define SERVO_PIN 32
// #define TEST_TUBE_DROPPER_SERVO_PIN 41

// ================= DHT sensor pins must be defined in DHTSensor.h ==================


MPU6050_Sensor mpu_sensor;
VL53L0X_Sensor vl53l0x;
BMP280_Sensor bmp_sensor;
// MQ2Sensor mq2(A4);
// MQ4Sensor mq4(A4);
// MQ6Sensor mq6(A2);
// MQ135Sensor mq135(MQ135_ANALOG_PIN);
MQ135_Digital mq135_digital(MQ135_DIGITAL_PIN);
// PHSensor ph_sensor(PH_ANALOG_PIN);
NPK_MB_Sensor npk_sensor; // pins to be set in header file
// TCS3200_Sensor tcs3200_sensor(S0, S1, S2, S3, OUT_PIN);
DHTSensor dht_sensor(DHT_PIN, DHT22_TYPE); // DHT sensor on pin 12
CurrentSensor current_sensor(ACS_PIN);

LinearActuator linear_actuator(LA_STEP_PIN, LA_DIR_PIN, LA_ENABLE_PIN, IS_LA_TB6600);
DrillMotor drill_motor(MOTOR_PWM, MOTOR_DIR, MOTOR_SLP);
BarrelMotor barrel_motor(B_STEP_PIN, B_DIR_PIN, B_ENABLE_PIN, IS_BARELL_TB6600);
PHServo ph_servo(SERVO_PIN);
// TestTubeDropper test_tube_dropper(TEST_TUBE_DROPPER_SERVO_PIN);

PubSub pubsub;

uint16_t distance, initial_distance_just_after_starting_drill = 0, dist_bw_sensor_drill = 5, cm_to_drill = 10, depth = 0;
int16_t ax, ay, az;
int16_t gx, gy, gz;
// thresholds for shaking detection
int16_t ax_th_x_min = -1000, ay_th_y_min = -1000, az_th_z_min = -1000;
int16_t ax_th_x_max = 1000, ay_th_y_max = 1000, az_th_z_max = 1000;
int16_t gx_th_x_min = -1000, gy_th_y_min = -1000, gz_th_z_min = -1000;
int16_t gx_th_x_max = 1000, gy_th_y_max = 1000, gz_th_z_max = 1000;
float pH, humidity, drill_motor_ma, drill_motor_ma_threshold_1 = 500.0f, drill_motor_ma_threshold_2 = 700.0f;
uint16_t nitrogen;
uint16_t phosphorus;
uint16_t potassium;


//  warning and other flags
bool currentServoToggleState = false;
bool ph_servo_position = false; // false = up, true = down
bool scienceModeEnable = false;
bool isBFirstPressed = false;
bool isStartDrillFlagged = false;
bool isStopDrillFlagged = false;
bool isShakeFlagged = false;
bool isCurrentThreshold1ExceededFlagged = false;
bool isCurrentThreshold2ExceededFlagged = false;
bool isDrillHalted = true;
bool isCO2Detected = false;
int messageCode = 0;

int delayCounter = 0;
int drillCooldownTimer = 1000; // while changing also change in loop where the value is being reset

void setup() {
  Serial.begin(115200);

#ifndef DEBUG_MODE
  pubsub.init();
#endif

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
//   tcs3200_sensor.begin();
//   mq2.begin();
//   mq4.begin();
//   mq6.begin();
//  mq135.begin();

  mq135_digital.begin(); // no callibiration needed for digital MQ135

  // println("Preheating MQ sensors for 20 seconds");
  // delay(20000); // Preheat

//   mq2.calibrate();
//   mq4.calibrate();
//   mq6.calibrate();
  // println("MQ sensor preheat complete, calibrating sensors...");
  // mq135.calibrate();
  // println("Calibration complete");
#endif

#ifdef ACTUATOR_ONLY_MODE
  println("=== Keyboard Control Ready ===");
  println("↑ : Clockwise");
  println("↓ : Anti-clockwise");
  println("← : Drill Motor decrease speed");
  println("→ : Drill Motor increase speed");
  println("a: Abruptly Stop Drill Motor");
  println("q: Drill Motor Clockwise");
  println("e: Drill Motor Anti-clockwise");
  println("f: Barrel Motor Rotate forward");
  println("b: Barrel Motor Rotate backward");
  println("s: Toggle PH Servo Position");
  // println("d: Drop Test Tube");
  println("0: Full | 1: 1/4 | 2: 1/8 | 3: 1/16");
#elif defined(DEBUG_MODE) && !defined(SENSOR_ONLY_MODE)
  println("=== Keyboard Control Ready ===");
  println("y: Toggle Science exploration mode");
  println("↑ : Clockwise");
  println("↓ : Anti-clockwise");
  println("← : Drill Motor decrease speed");
  println("→ : Drill Motor increase speed");
  println("a: Abruptly Stop Drill Motor");
  println("q: Drill Motor Clockwise");
  println("e: Drill Motor Anti-clockwise");
  println("f: Barrel Motor Rotate forward");
  println("b: Barrel Motor Rotate backward");
  println("s: Toggle PH Servo Position");
  // println("d: Drop Test Tube");
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
    print("Distance: ");
    print(distance);
    println(" mm");
  } else {
    println("Out of range");
  }

  print("Temperature: ");
  print(bmp_sensor.readTemperature());
  println(" °C");

  humidity = dht_sensor.getHumidity();
  if (humidity == -999.0f) {
        println("❌ DHT read error");
  } else {
      print("Humidity: ");
      print(humidity);
      println(" %");
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

  // print("MQ-135 CO2: "); print(mq135.readCO2()); println(" ppm");
  print("MQ-135 CO2 (Digital): ");
  print(mq135_digital.isCO2Detected());

  // pH = ph_sensor.readPH();
  // print("pH Value: ");
  // println(pH, 2);

//   if(tcs3200_sensor.isColorViolet()) {
//     println("TCS3200: Violet Color Detected");
//     } else {
//     println("TCS3200: Color Not Violet");
//     }
//   if(tcs3200_sensor.isColorless()) {
//       println("TCS3200: Colorless Detected");
//       } 
//     else {
//       println("TCS3200: Color Not Colorless");
//     }

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
  barrel_motor.run(); // rotate the motor asycnhronously, handles cases when motor not in rotation too
  linear_actuator.run();
  // print("Current (mA): ");
  // println(current_sensor.readCurrent());
//   delay(1000); // comment it out to while testing motord
  if (Serial.available()) {
    char c = Serial.read();

    // normal keys
    switch (c)
    {
        // drill motor controls
        case 'q': // CLOCKWISE
          drill_motor.changeDirection(CLOCKWISE);
          println("drill Clockwise");
          break;
        case 'e': // ANTICLOCKWISE
          drill_motor.changeDirection(ANTICLOCKWISE);
          println("drill Anti-clockwise");
          break;
        case 'a': // Abruptly stop drill motor
          drill_motor.stopMotor();
          println("Drill Motor Abruptly Stopped");
          break;

        // Barrel Motor Rotate
        case 'f':
            println("Barrel Motor Rotate 240 degrees forward");
            barrel_motor.handleBarellRotateInput(true);
        case 'b': 
          println("Barrel Motor Rotate 60 degrees backward");
          barrel_motor.handleBarellRotateInput(false);
          break;
        // PH Servo Toggle
        case 's':
        case 'S':
          println("Toggling PH Servo Position");
          ph_servo.togglePosition();
          break;
        // // drop the test tube
        // case 'd':
        //   println("Dropping Test Tube");
        //   test_tube_dropper.toggle();
        //   break;
    }

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
      switch (c) {
        // Linear Actuator controls
        case 'A': // Up Arrow
            if(!linear_actuator.motorRunning)
            {
                println("LA Rotate CW");
                // linear_actuator.rotateMotor(true, linear_actuator.stepsPerMove);
                linear_actuator.moveMotor(true);
                break;
            }
        case 'B': // Down Arrow
            if(!linear_actuator.motorRunning)
            {
                println("LA Rotate CCW");
                // linear_actuator.rotateMotor(false, linear_actuator.stepsPerMove);
                linear_actuator.moveMotor(false);
                break;
            }

        // drill motor speed controls
        case 'C': // Right
          drill_motor.increaseSpeed();
          print("drill Speed: ");
          println(drill_motor.targetSpeed);
          break;

        case 'D': // left
          drill_motor.decreaseSpeed();
          print("drill Speed: ");
          println(drill_motor.targetSpeed);
          break;
      }
    }
  }
#elif defined(DEBUG_MODE)
  static uint8_t escState = 0;
  barrel_motor.run();
  linear_actuator.run();
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
        isCurrentThreshold1ExceededFlagged = false;
        isCurrentThreshold2ExceededFlagged = false;
        isDrillHalted = true;
        initial_distance_just_after_starting_drill = 0;
        currentServoToggleState = false;
        isCO2Detected = false;
      }
      return;
    }

    if (!scienceModeEnable) {
      return; // Ignore other inputs if science mode is not enabled
    }
    //All controls
    switch (c) {
      // Linear Actuator controls
      // case 'A': // Up Arrow
      //   if(!linear_actuator.motorRunning)
      //   {
      //       println("LA Rotate ACW");
      //       // linear_actuator.rotateMotor(true, linear_actuator.stepsPerMove);
      //       linear_actuator.moveMotor(true);
      //   }
      //   break;
      // case 'B': // Down Arrow
      //   if(!linear_actuator.motorRunning)
      //   {
      //       println("LA Rotate CW");
      //       // linear_actuator.rotateMotor(false, linear_actuator.stepsPerMove);
      //       linear_actuator.moveMotor(false);
      //   }
      //   break;
      // drill motor controls
      // case 'Q':
      case 'q': // CLOCKWISE
        drill_motor.changeDirection(CLOCKWISE);
        println("drill Clockwise");
        break;
      // case 'E':
      case 'e': // ANTICLOCKWISE
        drill_motor.changeDirection(ANTICLOCKWISE);
        println("drill Anti-clockwise");
        break;
      case 'a': // Abruptly stop drill motor
        drill_motor.stopMotor();
        println("Drill Motor Abruptly Stopped");
        isDrillHalted = true;
        break;
      // Barrel Motor Rotate
        case 'f':
            println("Barrel Motor Rotate 240 degrees forward");
            isBFirstPressed = true;
            barrel_motor.handleBarellRotateInput(true);
            break;
      case 'b':
        println("Barrel Motor Rotate 240 degrees backward");
        isBFirstPressed = true;
        barrel_motor.handleBarellRotateInput(false);
        break;
      // PH Servo Toggle
      case 's':
      case 'S':
        if (isBFirstPressed)
        {
          println("Toggling PH Servo Position");
          ph_servo.togglePosition();
          currentServoToggleState = !currentServoToggleState;
        }
        break;
      // // test tube drop
      // case 'd':
      //   if(isBFirstPressed)
      //   {
      //     println("Dropping Test Tube");
      //     test_tube_dropper.toggle();
      //   }
    }

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
      switch (c) {
        // Linear Actuator controls
        case 'A': // Up Arrow
            if(!linear_actuator.motorRunning)
            {
                println("LA Rotate CW");
                // linear_actuator.rotateMotor(true, linear_actuator.stepsPerMove);
                linear_actuator.moveMotor(true);
                break;
            }
        case 'B': // Down Arrow
            if(!linear_actuator.motorRunning)
            {
                println("LA Rotate CCW");
                // linear_actuator.rotateMotor(false, linear_actuator.stepsPerMove);
                linear_actuator.moveMotor(false);
                break;
            }

        // drill motor speed controls
        case 'C': // Right
          if (!isCurrentThreshold2ExceededFlagged)
          {
            drill_motor.increaseSpeed();
            print("drill Speed: ");
            println(drill_motor.targetSpeed);
            isDrillHalted = false;
          }
          else
          {
            println("Drill motor speed change blocked due to overcurrent condition");
          }
          break;
        case 'D': // left
          if (!isCurrentThreshold2ExceededFlagged)
          {
            drill_motor.decreaseSpeed();
            print("drill Speed: ");
            println(drill_motor.targetSpeed);
            isDrillHalted = false;
          }
          else
          {
            println("Drill motor speed change blocked due to overcurrent condition");
          }
          break;
      }
    }

    // return;
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
      messageCode = 1; // start drilling
    }

    depth = initial_distance_just_after_starting_drill - distance;

    if(initial_distance_just_after_starting_drill - distance >= cm_to_drill*10 && isStartDrillFlagged && !isStopDrillFlagged) {
      println("WARNING: stop drilling");
      isStopDrillFlagged = true;
      messageCode = 2; // stop drilling
    }

    if((ax < ax_th_x_min || ax > ax_th_x_max ||
        ay < ay_th_y_min || ay > ay_th_y_max ||
        az < az_th_z_min || az > az_th_z_max ||
        gx < gx_th_x_min || gx > gx_th_x_max ||
        gy < gy_th_y_min || gy > gy_th_y_max ||
        gz < gz_th_z_min || gz > gz_th_z_max) && !isShakeFlagged) {
      println("ALERT: Shake Detected!");
      messageCode = 3; // shake detected
      isShakeFlagged = true;
    }

    if(drill_motor_ma > drill_motor_ma_threshold_1 && !isCurrentThreshold1ExceededFlagged) {
      println("ALERT: Current Threshold Exceeded!");
      isCurrentThreshold1ExceededFlagged = true;
      messageCode = 4; // current threshold 1 exceeded
    }

    if (drill_motor_ma > drill_motor_ma_threshold_2 && !isCurrentThreshold2ExceededFlagged) {
      println("CRITICAL ALERT: Current Threshold 2 Exceeded! stooping drill motor");
      drill_motor.stopMotor();
      isCurrentThreshold2ExceededFlagged = true;
      isDrillHalted = true;
      messageCode = 5; // current threshold 2 exceeded
    }

    if (isCurrentThreshold2ExceededFlagged && drillCooldownTimer > 0) drillCooldownTimer--;
    else
    {
      isCurrentThreshold2ExceededFlagged = false;
      drillCooldownTimer = 1000;
    }
    println("--------------------------");
    println("Current sensor reading (mA): " + String(drill_motor_ma));
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

    // print("MQ-135 CO2: "); print(mq135.readCO2()); println(" ppm");
    if (!isCO2Detected && mq135_digital.isCO2Detected()) {
      println("ALERT: MQ-135 CO2 Detected!");
      isCO2Detected = true;
    }
    print("MQ-135 CO2 (Digital): ");
    print(mq135_digital.isCO2Detected());

    // pH = ph_sensor.readPH();
    // print("pH Value: ");
    // println(pH, 2);

    // if(tcs3200_sensor.isColorViolet()) {
    //   println("TCS3200: Violet Color Detected");
    //   } 
    // else {
    //   println("TCS3200: Color Not Violet");
    // }
    // if(tcs3200_sensor.isColorless()) {
    //   println("TCS3200: Colorless Detected");
    //   } 
    // else {
    //   println("TCS3200: Color Not Colorless");
    // }

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
  pubsub.handle_subscriptions();
  barrel_motor.run();
  linear_actuator.run();
  // publish the command recieved from base station for debugging purpose
  pubsub.publish_cmd_received(
      linear_actuator_cmd,
      drill_cmd,
      barrel_cmd,
      servo_toggle,
      science_module_toggle
  );
  if (science_module_toggle == 1) scienceModeEnable = true;
  else if(science_module_toggle == 0) scienceModeEnable = false;

  if (scienceModeEnable) {
        println("Science Exploration Mode Enabled");
  } 
  else {
    isBFirstPressed = false;
    isStartDrillFlagged = false;
    isStopDrillFlagged = false;
    isShakeFlagged = false;
    isCurrentThreshold1ExceededFlagged = false;
    isCurrentThreshold2ExceededFlagged = false;
    isDrillHalted = true;
    ph_servo_position = false;
    isCO2Detected = false;
    initial_distance_just_after_starting_drill = 0;
    println("Science Exploration Mode Disabled");
    return;
  }
  // ============ All controls ==================
  // Linear Actuator controls
  if (!linear_actuator.motorRunning)
  {
    if(linear_actuator_cmd == 1) // Up Arrow
    {
        // linear_actuator.rotateMotor(true, linear_actuator.stepsPerMove);
        linear_actuator.moveMotor(true);
        println("LA Rotate ACW");
    }
    else if(linear_actuator_cmd == -1) // Down Arrow
    {
        // linear_actuator.rotateMotor(false, linear_actuator.stepsPerMove);
        linear_actuator.moveMotor(false);
        println("LA Rotate CW");
    }
  }
//   if(linear_actuator_cmd == 0) linear_actuator.setMicrostepping(FULL);
//   else if(linear_actuator_cmd == 4) linear_actuator.setMicrostepping(MICRO_1_4);
//   else if(linear_actuator_cmd == 8) linear_actuator.setMicrostepping(MICRO_1_8);
//   else if(linear_actuator_cmd == 16) linear_actuator.setMicrostepping(MICRO_1_16);
  // drill motor controls
  if(drill_cmd == -2) // CLOCKWISE
  {
    drill_motor.changeDirection(CLOCKWISE);
    println("drill Clockwise");
  }
  else if(drill_cmd == 2) // ANTICLOCKWISE
  {
    drill_motor.changeDirection(ANTICLOCKWISE);
    println("drill Anticlockwise");
  }

  if(drill_cmd == 1) // Right
  {
    if (!isCurrentThreshold2ExceededFlagged)
    {
      drill_motor.increaseSpeed();
      isDrillHalted = false;
      println("drill Speed Increased to " + String(drill_motor.targetSpeed));
    }
  }
  else if(drill_cmd == -1) // left
  {
    if (!isCurrentThreshold2ExceededFlagged)
    {
      drill_motor.decreaseSpeed();
      isDrillHalted = false;
      println("drill Speed Decreased to " + String(drill_motor.targetSpeed));
    }
  }
  else if(drill_cmd == 0) // Abruptly stop drill motor
  {
    drill_motor.stopMotor();
    isDrillHalted = true;
    println("Drill Motor Abruptly Stopped");
  }

  // Barrel Motor Rotate
  if(barrel_cmd == 1) // forward
  {
    isBFirstPressed = true;
    barrel_motor.handleBarellRotateInput(true);
    println("Barrel Motor Rotate Forward");
  }
  else if(barrel_cmd == -1) // backward
  {
    isBFirstPressed = true;
    barrel_motor.handleBarellRotateInput(false);
    println("Barrel Motor Rotate Backward");
  }

  // // drop test tube
  // if(servo_toggle == 2 && isBFirstPressed){ 
  //   test_tube_dropper.toggle();
  //   println("Dropping Test Tube");
  // }

  // PH Servo Toggle
  if(servo_toggle == 1) ph_servo_position = true;
  else if(servo_toggle == 0) ph_servo_position = false;
  if (ph_servo_position != currentServoToggleState && isBFirstPressed)
  {
    ph_servo.togglePosition();
    currentServoToggleState = ph_servo_position;
  }

  // ========== reset commands ==========
  linear_actuator_cmd = LINEAR_ACTUATOR_CMD_DEFAULT;
  barrel_cmd = BARREL_CMD_DEFAULT;
  drill_cmd = DRILL_CMD_DEFAULT;

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
      pubsub.publish_info_warning(1); // publish start drilling info
      println("INFO: start drilling");
      initial_distance_just_after_starting_drill = distance;
      isStartDrillFlagged = true;
      messageCode = 1; // start drilling
    }

    depth = initial_distance_just_after_starting_drill - distance;

    if(initial_distance_just_after_starting_drill - distance >= cm_to_drill*10 && isStartDrillFlagged && !isStopDrillFlagged) {
      pubsub.publish_info_warning(2); // publish stop drilling warning
      println("WARNING: stop drilling");
      isStopDrillFlagged = true;
      messageCode = 2; // stop drilling
    }

    if((ax < ax_th_x_min || ax > ax_th_x_max ||
        ay < ay_th_y_min || ay > ay_th_y_max ||
        az < az_th_z_min || az > az_th_z_max ||
        gx < gx_th_x_min || gx > gx_th_x_max ||
        gy < gy_th_y_min || gy > gy_th_y_max ||
        gz < gz_th_z_min || gz > gz_th_z_max) && !isShakeFlagged) {
      pubsub.publish_info_warning(3); // publish shake detected alert
      println("ALERT: Shake Detected!");
      messageCode = 3; // shake detected
      isShakeFlagged = true;
    }

    if(drill_motor_ma > drill_motor_ma_threshold_1 && !isCurrentThreshold1ExceededFlagged) {
      pubsub.publish_info_warning(4); // publish current threshold 1 exceeded alert
      println("ALERT: Current Threshold 1 Exceeded!");
      isCurrentThreshold1ExceededFlagged = true;
      messageCode = 4; // current threshold 1 exceeded
    }

    if (drill_motor_ma > drill_motor_ma_threshold_2 && !isCurrentThreshold2ExceededFlagged) {
      pubsub.publish_info_warning(5); // publish current threshold 2 exceeded alert
      drill_motor.stopMotor();
      println("CRITICAL ALERT: Current Threshold 2 Exceeded! stopping drill motor");
      isCurrentThreshold2ExceededFlagged = true;
      isDrillHalted = true;
      messageCode = 5; // current threshold 2 exceeded
    }

    if (isCurrentThreshold2ExceededFlagged && drillCooldownTimer > 0) drillCooldownTimer--;
    else
    {
      isCurrentThreshold2ExceededFlagged = false;
      println("INFO: Current Threshold 2 Cooldown Complete");
      drillCooldownTimer = 1000;
    }

    pubsub.publish_drill_data(isDrillHalted, distance, ax, ay, az, gx, gy, gz);
  }

  if (!isCO2Detected && mq135_digital.isCO2Detected()) {
      println("ALERT: MQ-135 CO2 Detected!");
      isCO2Detected = true;
    }

  if (isBFirstPressed && delayCounter > DELAY_BETWEEN_SENSOR_READS) {
    pubsub.publish_sensor_data(
      -6, //tcs3200_sensor.isColorless(), 
      -6, //tcs3200_sensor.isColorViolet(), 
      dht_sensor.getHumidity(), nitrogen, phosphorus, potassium, 
      0, //ph placeholder
      // mq135.readCO2(), 
      isCO2Detected ? 1.0 : 0.0,
      bmp_sensor.readTemperature(), 
      bmp_sensor.readPressure(), bmp_sensor.readAltitude(),
      depth, 
      0,0 // lat and long placeholder
    );
  }

  if (delayCounter > DELAY_BETWEEN_SENSOR_READS) {
    delayCounter = 0;
  }
#endif
}

