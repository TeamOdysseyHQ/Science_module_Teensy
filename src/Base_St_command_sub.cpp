// #include <micro_ros_arduino.h>
// #include <rcl/rcl.h>
// #include <rclc/rclc.h>
// #include <rclc/executor.h>
// #include <std_msgs/msg/int32_multi_array.h>

// rcl_node_t node;
// rcl_subscription_t subscriber;
// rclc_executor_t executor;
// rcl_allocator_t allocator;
// rclc_support_t support;

// std_msgs__msg__Int32MultiArray msg;
// int32_t data_buffer[5];

// void subscription_callback(const void *msgin)
// {
//   const std_msgs__msg__Int32MultiArray *msg =
//     (const std_msgs__msg__Int32MultiArray *)msgin;

//   Serial.println("Received Science Command:");

//   if (msg->data.data[0] == 1) Serial.println("Drill: UP");
//   else if (msg->data.data[0] == -1) Serial.println("Drill: DOWN");

//   if (msg->data.data[1] == 1) Serial.println("Drill Speed: INCREASE");
//   else if (msg->data.data[1] == -1) Serial.println("Drill Speed: DECREASE");

//   if (msg->data.data[2] == 1) Serial.println("Barrel: ACTIVATED");

//   if (msg->data.data[3] == 1) Serial.println("Servo: TOGGLE");

//   if (msg->data.data[4] == 1) Serial.println("Science Exploration: TOGGLE");

//   Serial.println("----------------------");
// }

// void setup()
// {
//   Serial.begin(115200);

//   // WAIT for USB CDC to be ready (CRITICAL)
//   while (!Serial && millis() < 8000) {
//     delay(10);
//   }

//   delay(2000);

//   set_microros_transports();

//   allocator = rcl_get_default_allocator();
//   rclc_support_init(&support, 0, NULL, &allocator);

//   rclc_node_init_default(&node, "teensy_science_subscriber", "", &support);

//   rclc_subscription_init_default(
//     &subscriber,
//     &node,
//     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
//     "/science_control");

//   msg.data.data = data_buffer;
//   msg.data.size = 5;
//   msg.data.capacity = 5;

//   rclc_executor_init(&executor, &support.context, 1, &allocator);
//   rclc_executor_add_subscription(
//     &executor,
//     &subscriber,
//     &msg,
//     &subscription_callback,
//     ON_NEW_DATA);
// }

// void loop()
// {
//   rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
// }
