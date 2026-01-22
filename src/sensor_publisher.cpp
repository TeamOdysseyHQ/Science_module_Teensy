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
