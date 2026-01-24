#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/int32.h>

// =================================================
// ================= ROS OBJECTS ===================
// =================================================
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

// =================================================
// ================= PUBLISHERS ====================
// =================================================
rcl_publisher_t drill_publisher;
rcl_publisher_t sensor_publisher;
rcl_publisher_t info_warning_publisher;

// =================================================
// ================= MESSAGES ======================
// =================================================
std_msgs__msg__Float32MultiArray drill_msg;
std_msgs__msg__Float32MultiArray sensor_msg;
std_msgs__msg__Int32 info_warning_msg;

// =================================================
// ================= BUFFERS =======================
// =================================================
#define DRILL_ARRAY_SIZE 8
float drill_data_buffer[DRILL_ARRAY_SIZE];

#define SENSOR_ARRAY_SIZE 14
float sensor_data_buffer[SENSOR_ARRAY_SIZE];

int32_t info_warning_buffer;

// =================================================
// ============ PUBLISH FUNCTIONS ==================
// =================================================

// -------- CONTINUOUS DRILL DATA --------
void publish_drill_data(
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
void publish_sensor_data(
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
void publish_info_warning(int32_t code) {
  info_warning_buffer = code;
  info_warning_msg.data = info_warning_buffer;

  rcl_publish(&info_warning_publisher, &info_warning_msg, NULL);
}

void setup() {
  set_microros_transports();
  delay(2000);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);

  rclc_node_init_default(
    &node,
    "teensy_science_node",
    "",
    &support
  );

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
}

void loop() {
  publish_drill_data(
    false,
    random(-250, 250),
    random(-250, 250) / 10.0, random(-250, 250) / 10.0, random(-250, 250) / 10.0,
    random(-250, 250) / 10.0, random(-250, 250) / 10.0, random(-250, 250) / 10.0
  );

  publish_sensor_data(
    true,   // colorless
    false,  // purple
    random(-250, 250) / 10.0,   // humidity
    random(-250, 250), random(-250, 250), random(-250, 250), // NPK
    random(-250, 250) / 10.0,    // pH
    random(-250, 250),    // CO2
    random(-250, 250) / 10.0,   // temp
    random(-250, 250), // pressure
    random(-250, 250) / 10.0,  // altitude
    random(0, 250),   // depth
    0, // latitude
    0  // longitude
  );

  publish_info_warning(random(-10, 10));

  delay(1000);
}
