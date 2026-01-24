// // #include <micro_ros_arduino.h>

// // #include <rcl/rcl.h>
// // #include <rclc/rclc.h>
// // #include <rclc/executor.h>

// // #include <std_msgs/msg/float32_multi_array.h>
// // #include <std_msgs/msg/int32.h>
// // #include <std_msgs/msg/int32_multi_array.h>

// // // =================================================
// // // ================= ROS OBJECTS ===================
// // // =================================================
// // rcl_allocator_t allocator;
// // rclc_support_t support;
// // rcl_node_t node;
// // rclc_executor_t executor;

// // // =================================================
// // // ================= PUBLISHERS ====================
// // // =================================================
// // rcl_publisher_t drill_publisher;
// // rcl_publisher_t sensor_publisher;
// // rcl_publisher_t info_warning_publisher;

// // // =================================================
// // // ================ SUBSCRIBERS ====================
// // // =================================================
// // rcl_subscription_t control_subscriber;

// // // =================================================
// // // ================= MESSAGES ======================
// // // =================================================
// // std_msgs__msg__Float32MultiArray drill_msg;
// // std_msgs__msg__Float32MultiArray sensor_msg;
// // std_msgs__msg__Int32 info_warning_msg;
// // std_msgs__msg__Int32MultiArray control_msg;
// // // =================================================
// // // ================= BUFFERS =======================
// // // =================================================
// // #define DRILL_ARRAY_SIZE 8
// // float drill_data_buffer[DRILL_ARRAY_SIZE];

// // #define SENSOR_ARRAY_SIZE 14
// // float sensor_data_buffer[SENSOR_ARRAY_SIZE];

// // int32_t info_warning_buffer;

// // int32_t command_data_buffer[5];

// // // variables to access the commands
// // volatile int8_t linear_actuator_cmd = 0;
// // volatile int8_t drill_cmd = 0;
// // volatile bool barrel_active = false;
// // volatile bool servo_toggle = false;
// // volatile bool science_module_toggle = false;

// // // =================================================
// // // ============ PUBLISH FUNCTIONS ==================
// // // =================================================

// // // -------- CONTINUOUS DRILL DATA --------
// // void publish_drill_data(
// //   bool drill_halted,
// //   float distance,
// //   float ax, float ay, float az,
// //   float gx, float gy, float gz
// // ) {
// //   drill_data_buffer[0] = drill_halted ? 1.0f : 0.0f;
// //   drill_data_buffer[1] = distance;
// //   drill_data_buffer[2] = ax;
// //   drill_data_buffer[3] = ay;
// //   drill_data_buffer[4] = az;
// //   drill_data_buffer[5] = gx;
// //   drill_data_buffer[6] = gy;
// //   drill_data_buffer[7] = gz;

// //   rcl_publish(&drill_publisher, &drill_msg, NULL);
// // }

// // // -------- ON-DEMAND SENSOR DATA --------
// // void publish_sensor_data(
// //   bool color_colorless,
// //   bool color_purple,
// //   float humidity,
// //   float n, float p, float k,
// //   float ph,
// //   float co2,
// //   float temperature,
// //   float pressure,
// //   float altitude,
// //   float depth,
// //   float latitude,
// //   float longitude
// // ) {
// //   sensor_data_buffer[0]  = color_colorless ? 1.0f : 0.0f;
// //   sensor_data_buffer[1]  = color_purple ? 1.0f : 0.0f;
// //   sensor_data_buffer[2]  = humidity;
// //   sensor_data_buffer[3]  = n;
// //   sensor_data_buffer[4]  = p;
// //   sensor_data_buffer[5]  = k;
// //   sensor_data_buffer[6]  = ph;
// //   sensor_data_buffer[7]  = co2;
// //   sensor_data_buffer[8]  = temperature;
// //   sensor_data_buffer[9]  = pressure;
// //   sensor_data_buffer[10] = altitude;
// //   sensor_data_buffer[11] = latitude;
// //   sensor_data_buffer[12] = longitude;
// //   sensor_data_buffer[13] = depth;
  

// //   rcl_publish(&sensor_publisher, &sensor_msg, NULL);
// // }

// // // ----- Info/Warning Publisher -----
// // void publish_info_warning(int32_t code) {
// //   info_warning_buffer = code;
// //   info_warning_msg.data = info_warning_buffer;

// //   rcl_publish(&info_warning_publisher, &info_warning_msg, NULL);
// // }

// // // =================================================
// // // ============== SUBSCRIPTION CALLBACK ==============
// // // =================================================
// // void subscription_callback(const void *msgin)
// // {
// //   const std_msgs__msg__Int32MultiArray *control_msg =
// //     (const std_msgs__msg__Int32MultiArray *)msgin;

// //   linear_actuator_cmd = control_msg->data.data[0];
// //   drill_cmd = control_msg->data.data[1];
// //   barrel_active = control_msg->data.data[2] == 1 ? true : false;
// //   servo_toggle = control_msg->data.data[3] == 1 ? true : false;
// //   science_module_toggle = control_msg->data.data[4] == 1 ? true : false;
// // }

// // void command_test()
// // {
// //     bool led_on = false;
// //     if (linear_actuator_cmd == 1) {
// //       led_on = true;
// //       linear_actuator_cmd = 0;
// //     }
// //     if (drill_cmd == 1) {
// //       led_on = true;
// //       drill_cmd = 0;
// //     }
// //     if (barrel_active) {
// //       led_on = true;
// //       barrel_active = false;
// //     }
// //     if (servo_toggle) {
// //       led_on = true;
// //       servo_toggle = false;
// //     }
// //     if (science_module_toggle) {
// //       led_on = true;
// //       science_module_toggle = false;
// //     }

// //     if (led_on)
// //     {
// //       digitalWrite(LED_BUILTIN, HIGH);
// //       delay(500);
// //       digitalWrite(LED_BUILTIN, LOW);
// //     }
// // }

// // void setup() {
// //   set_microros_transports();
// //   pinMode(LED_BUILTIN, OUTPUT);
// //   delay(2000);

// //   allocator = rcl_get_default_allocator();
// //   rclc_support_init(&support, 0, NULL, &allocator);

// //   rclc_node_init_default(
// //     &node,
// //     "teensy_science_node",
// //     "",
// //     &support
// //   );

// //   rclc_executor_init(&executor, &support.context, 1, &allocator);

// //   // ---- DRILL publisher ----
// //   rclc_publisher_init_default(
// //     &drill_publisher,
// //     &node,
// //     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
// //     "science_drill_data"
// //   );

// //   drill_msg.data.data = drill_data_buffer;
// //   drill_msg.data.size = DRILL_ARRAY_SIZE;
// //   drill_msg.data.capacity = DRILL_ARRAY_SIZE;

// //   // ---- SENSOR publisher (on-demand) ----
// //   rclc_publisher_init_default(
// //     &sensor_publisher,
// //     &node,
// //     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
// //     "science_sensor_data"
// //   );

// //   sensor_msg.data.data = sensor_data_buffer;
// //   sensor_msg.data.size = SENSOR_ARRAY_SIZE;
// //   sensor_msg.data.capacity = SENSOR_ARRAY_SIZE;

// //   // ---- INFO/WARNING publisher ----
// //   rclc_publisher_init_default(
// //     &info_warning_publisher,
// //     &node,
// //     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
// //     "science_info_warning"
// //   );

// //   info_warning_msg.data = info_warning_buffer;

// //   // ==== SUBSCRIBER ======
// //   rclc_subscription_init_default(
// //     &control_subscriber,
// //     &node,
// //     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
// //     "/science_control");

// //   control_msg.data.data = command_data_buffer;
// //   control_msg.data.size = 5;
// //   control_msg.data.capacity = 5;

// //   // function to be called automatically with internal timer
// //   rclc_executor_add_subscription(
// //     &executor,
// //     &control_subscriber,
// //     &control_msg,
// //     &subscription_callback,
// //     ON_NEW_DATA);
// // }


// // int counter = 0;
// // void loop() {
// //   if (counter++ > 1000)
// //   {
// //     counter = 0;
// //     publish_drill_data(
// //       false,
// //       random(-250, 250),
// //       random(-250, 250) / 10.0, random(-250, 250) / 10.0, random(-250, 250) / 10.0,
// //       random(-250, 250) / 10.0, random(-250, 250) / 10.0, random(-250, 250) / 10.0
// //     );

// //     publish_sensor_data(
// //       true,   // colorless
// //       false,  // purple
// //       random(-250, 250) / 10.0,   // humidity
// //       random(-250, 250), random(-250, 250), random(-250, 250), // NPK
// //       random(-250, 250) / 10.0,    // pH
// //       random(-250, 250),    // CO2
// //       random(-250, 250) / 10.0,   // temp
// //       random(-250, 250), // pressure
// //       random(-250, 250) / 10.0,  // altitude
// //       random(0, 250),   // depth
// //       0, // latitude
// //       0  // longitude
// //     );

// //     publish_info_warning(random(-10, 10));
// //   }

// //   // call the subscription callback which checks for new messages automatically
// //   rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
// //   command_test();
// // }

// #include "PubSub.h"

// PubSub pubsub;

// void command_test()
// {
//     bool led_on = false;
//     if (linear_actuator_cmd == 1) {
//       led_on = true;
//       linear_actuator_cmd = 0;
//     }
//     if (drill_cmd == 1) {
//       led_on = true;
//       drill_cmd = 0;
//     }
//     if (barrel_cmd == 1) {
//       led_on = true;
//       barrel_cmd = 0;
//     }
//     if (servo_toggle) {
//       led_on = true;
//       servo_toggle = false;
//     }
//     if (science_module_toggle) {
//       led_on = true;
//       science_module_toggle = false;
//     }

//     if (led_on)
//     {
//       digitalWrite(LED_BUILTIN, HIGH);
//       delay(500);
//       digitalWrite(LED_BUILTIN, LOW);
//     }
// }

// void setup() {
//   Serial.begin(115200);
//   pinMode(LED_BUILTIN, OUTPUT);
//   pubsub.init();
// }

// int counter = 0;
// void loop() {

//   if(counter++ > 1000)
//   {
//     counter = 0;
//     pubsub.publish_drill_data(
//         false,
//         random(-250, 250),
//         random(-250, 250) / 10.0, random(-250, 250) / 10.0, random(-250, 250) / 10.0,
//         random(-250, 250) / 10.0, random(-250, 250) / 10.0, random(-250, 250) / 10.0
//     );

//     pubsub.publish_sensor_data(
//         true,   // colorless
//         false,  // purple
//         random(-250, 250) / 10.0,   // humidity
//         random(-250, 250), random(-250, 250), random(-250, 250), // NPK
//         random(-250, 250) / 10.0,    // pH
//         random(-250, 250),    // CO2
//         random(-250, 250) / 10.0,   // temp
//         random(-250, 250), // pressure
//         random(-250, 250) / 10.0,  // altitude
//         random(0, 250),   // depth
//         0, // latitude
//         0  // longitude
//     );
//   }

//   pubsub.publish_info_warning(random(-10, 10));

//   pubsub.handle_subscriptions();
//   command_test();
// }