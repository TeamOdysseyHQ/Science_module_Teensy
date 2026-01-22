// /**
//  * @file sensor_publisher.cpp
//  * @brief Publishes sensor data using micro-ROS on a Teensy 4.1 Tested and works with optiplex
//  * @author Rtamanyu N J
//  * @date 2024-06-20
//  * Won't work in WSL (port config req) - use nativ eLinux
//  */

// #include <micro_ros_arduino.h>

// #include <rcl/rcl.h>
// #include <rclc/rclc.h>
// #include <rclc/executor.h>

// #include <std_msgs/msg/float32_multi_array.h>

// rcl_node_t node;
// rcl_publisher_t publisher;
// rcl_timer_t timer;
// rclc_executor_t executor;
// rclc_support_t support;
// rcl_allocator_t allocator;

// std_msgs__msg__Float32MultiArray msg;
// float data_buffer[3];

// void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
// {
//   (void) last_call_time;
//   if (timer == NULL) return;

//   // Example sensor values
//   data_buffer[0] = analogRead(A0) * 0.01;
//   data_buffer[1] = analogRead(A1) * 0.01;
//   data_buffer[2] = analogRead(A2) * 0.01;

//   msg.data.data = data_buffer;
//   msg.data.size = 3;
//   msg.data.capacity = 3;

//   rcl_publish(&publisher, &msg, NULL);
// }

// // void setup()
// // {
// //   set_microros_transports();   // USB Serial
// //   delay(2000);

// //   allocator = rcl_get_default_allocator();

// //   rclc_support_init(&support, 0, NULL, &allocator);

// //   // NODE with namespace
// //   rclc_node_init_default(
// //     &node,
// //     "teensy_science_sensor_publisher",
// //     "test_subscribe",
// //     &support
// //   );

// //   // PUBLISHER
// //   rclc_publisher_init_default(
// //     &publisher,
// //     &node,
// //     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
// //     "Science_Sensor_Data"
// //   );

// //   // TIMER (10 Hz)
// //   rclc_timer_init_default(
// //     &timer,
// //     &support,
// //     RCL_MS_TO_NS(100),
// //     timer_callback
// //   );

// //   rclc_executor_init(&executor, &support.context, 1, &allocator);
// //   rclc_executor_add_timer(&executor, &timer);
// // }

// // void loop()
// // {
// //   rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
// // }

#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32_multi_array.h>

rcl_publisher_t publisher;
std_msgs__msg__Float32MultiArray msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define ARRAY_SIZE 14

float data_buffer[ARRAY_SIZE];

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  (void) last_call_time;

  // Random boolean colors
  data_buffer[0] = random(0, 2);  // red
  data_buffer[1] = random(0, 2);  // purple
  data_buffer[2] = random(0, 2);  // pink

  // Random sensor values
  data_buffer[3]  = random(10, 80);      // N
  data_buffer[4]  = random(10, 80);      // P
  data_buffer[5]  = random(10, 80);      // K
  data_buffer[6]  = random(50, 90) / 10.0; // pH
  data_buffer[7]  = random(300, 1200);   // CO2 ppm
  data_buffer[8]  = random(200, 350) / 10.0; // Temp °C
  data_buffer[9]  = random(900, 1100);   // Pressure hPa
  data_buffer[10] = random(0, 200);      // Altitude m
  data_buffer[11] = 12.9716;             // Latitude
  data_buffer[12] = 77.5946;             // Longitude
  data_buffer[13] = random(10, 400);     // Distance cm

  rcl_publish(&publisher, &msg, NULL);
}

void setup()
{
  Serial.begin(115200);
  delay(2000);

  set_microros_transports();

  allocator = rcl_get_default_allocator();

  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "Science_publisher", "", &support);

  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "science_sensor_data"
  );

  msg.data.capacity = ARRAY_SIZE;
  msg.data.size = ARRAY_SIZE;
  msg.data.data = data_buffer;

  rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(1000),
    timer_callback
  );

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_timer(&executor, &timer);
}

void loop()
{
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
