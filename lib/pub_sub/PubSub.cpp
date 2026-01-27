#include "PubSub.h"

// variables to access the commands
volatile int8_t linear_actuator_cmd = LINEAR_ACTUATOR_CMD_DEFAULT;
volatile int8_t drill_cmd = DRILL_CMD_DEFAULT;
volatile int8_t barrel_cmd = BARREL_CMD_DEFAULT;
volatile int8_t servo_toggle = SERVO_TOGGLE_DEFAULT;
volatile int8_t science_module_toggle = SCIENCE_MODULE_TOGGLE_DEFAULT;
PubSub::PubSub() {}

// =================================================
// ============ PUBLISH FUNCTIONS ==================
// =================================================

// -------- CONTINUOUS DRILL DATA --------
void PubSub::publish_drill_data(
  bool drill_halted,
  float distance,
  float ax, float ay, float az,
  float gx, float gy, float gz
) {
  drill_data_buffer[0] = drill_halted ? 1.0f : 0.0f;
  drill_data_buffer[1] = distance;
  drill_data_buffer[2] = ax;
  drill_data_buffer[3] = ay;
  drill_data_buffer[4] = az;
  drill_data_buffer[5] = gx;
  drill_data_buffer[6] = gy;
  drill_data_buffer[7] = gz;

  rcl_publish(&drill_publisher, &drill_msg, NULL);
}

// -------- ON-DEMAND SENSOR DATA --------
void PubSub::publish_sensor_data(
  bool color_colorless,
  bool color_purple,
  float humidity,
  float n, float p, float k,
  float ph,
  float co2,
  float temperature,
  float pressure,
  float altitude,
  float depth,
  float latitude,
  float longitude
) {
  sensor_data_buffer[0]  = color_colorless ? 1.0f : 0.0f;
  sensor_data_buffer[1]  = color_purple ? 1.0f : 0.0f;
  sensor_data_buffer[2]  = humidity;
  sensor_data_buffer[3]  = n;
  sensor_data_buffer[4]  = p;
  sensor_data_buffer[5]  = k;
  sensor_data_buffer[6]  = ph;
  sensor_data_buffer[7]  = co2;
  sensor_data_buffer[8]  = temperature;
  sensor_data_buffer[9]  = pressure;
  sensor_data_buffer[10] = altitude;
  sensor_data_buffer[11] = latitude;
  sensor_data_buffer[12] = longitude;
  sensor_data_buffer[13] = depth;
  

  rcl_publish(&sensor_publisher, &sensor_msg, NULL);
}

// ----- Info/Warning Publisher -----
void PubSub::publish_info_warning(int32_t code) {
  info_warning_buffer = code;
  info_warning_msg.data = info_warning_buffer;

  rcl_publish(&info_warning_publisher, &info_warning_msg, NULL);
}

// ----- CMD RECEIVED PUBLISHER -----
void PubSub::publish_cmd_received(
    int8_t linear_actuator_cmd,
    int8_t drill_cmd,
    int8_t barrel_cmd,
    int8_t servo_toggle,
    int8_t science_module_toggle
) {
    cmd_received_buffer[0] = linear_actuator_cmd;
    cmd_received_buffer[1] = drill_cmd;
    cmd_received_buffer[2] = barrel_cmd;
    cmd_received_buffer[3] = servo_toggle;
    cmd_received_buffer[4] = science_module_toggle;

    rcl_publish(&cmd_received_publisher, &cmd_received_msg, NULL);
}

// =================================================
// ============== SUBSCRIPTION CALLBACK ==============
// =================================================
void subscription_callback(const void *msgin)
{
  const std_msgs__msg__Int32MultiArray *control_msg =
    (const std_msgs__msg__Int32MultiArray *)msgin;

  linear_actuator_cmd = control_msg->data.data[0];
  drill_cmd = control_msg->data.data[1];
  barrel_cmd = control_msg->data.data[2];
  servo_toggle = control_msg->data.data[3];
  science_module_toggle = control_msg->data.data[4];
}

void PubSub::handle_subscriptions() {
  // call the subscription callback which checks for new messages automatically
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}

void PubSub::init() {
  set_microros_transports();

  // 3️⃣ Retry agent connection
  while (rmw_uros_ping_agent(1000, 5) != RMW_RET_OK) {
    delay(1000);
  }

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);

  rclc_node_init_default(
    &node,
    "teensy_science_node",
    "",
    &support
  );

  rclc_executor_init(&executor, &support.context, 1, &allocator);

  // ---- DRILL publisher ----
  rclc_publisher_init_default(
    &drill_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "science_drill_data"
  );

  drill_msg.data.data = drill_data_buffer;
  drill_msg.data.size = DRILL_ARRAY_SIZE;
  drill_msg.data.capacity = DRILL_ARRAY_SIZE;

  // ---- SENSOR publisher (on-demand) ----
  rclc_publisher_init_default(
    &sensor_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "science_sensor_data"
  );

  sensor_msg.data.data = sensor_data_buffer;
  sensor_msg.data.size = SENSOR_ARRAY_SIZE;
  sensor_msg.data.capacity = SENSOR_ARRAY_SIZE;

  // ---- INFO/WARNING publisher ----
  rclc_publisher_init_default(
    &info_warning_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "science_info_warning"
  );

  info_warning_msg.data = info_warning_buffer;

  // ---- CMD RECEIVED publisher ----
  rclc_publisher_init_default(
    &cmd_received_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "science_cmd_received"
  );
  cmd_received_msg.data.data = cmd_received_buffer;
  cmd_received_msg.data.size = CMD_RECEIVED_ARRAY_SIZE;
  cmd_received_msg.data.capacity = CMD_RECEIVED_ARRAY_SIZE;

  // ==== SUBSCRIBER ======
  rclc_subscription_init_default(
    &control_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "/science_control");

  control_msg.data.data = command_data_buffer;
  control_msg.data.size = 5;
  control_msg.data.capacity = 5;

  // function to be called automatically with internal timer
  rclc_executor_add_subscription(
    &executor,
    &control_subscriber,
    &control_msg,
    &subscription_callback,
    ON_NEW_DATA);
}
