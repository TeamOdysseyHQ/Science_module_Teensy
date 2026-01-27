#ifndef PUB_SUB_H
#define PUB_SUB_H

#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int32_multi_array.h>

#define LINEAR_ACTUATOR_CMD_DEFAULT -6
#define DRILL_CMD_DEFAULT -6
#define BARREL_CMD_DEFAULT -6
#define SERVO_TOGGLE_DEFAULT -6
#define SCIENCE_MODULE_TOGGLE_DEFAULT -6

extern volatile int8_t linear_actuator_cmd;
extern volatile int8_t drill_cmd;
extern volatile int8_t barrel_cmd;
extern volatile int8_t servo_toggle;
extern volatile int8_t science_module_toggle;

class PubSub {
public:
    PubSub();
    void init();
    void publish_drill_data(
        bool drill_halted,
        float distance,
        float ax, float ay, float az,
        float gx, float gy, float gz
    );
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
    );
    void publish_info_warning(int32_t info_warning_code);

    void publish_cmd_received(
        int8_t linear_actuator_cmd,
        int8_t drill_cmd,
        int8_t barrel_cmd,
        int8_t servo_toggle,
        int8_t science_module_toggle
    );

    void handle_subscriptions();
private:
    // =================================================
    // ================= ROS OBJECTS ===================
    // =================================================
    rcl_allocator_t allocator;
    rclc_support_t support;
    rcl_node_t node;
    rclc_executor_t executor;

    // =================================================
    // ================= PUBLISHERS ====================
    // =================================================
    rcl_publisher_t drill_publisher;
    rcl_publisher_t sensor_publisher;
    rcl_publisher_t info_warning_publisher;

    rcl_publisher_t cmd_received_publisher;

    // =================================================
    // ================ SUBSCRIBERS ====================
    // =================================================
    rcl_subscription_t control_subscriber;

    // =================================================
    // ================= MESSAGES ======================
    // =================================================
    std_msgs__msg__Float32MultiArray drill_msg;
    std_msgs__msg__Float32MultiArray sensor_msg;
    std_msgs__msg__Int32 info_warning_msg;
    std_msgs__msg__Int32MultiArray control_msg;

    std_msgs__msg__Int32MultiArray cmd_received_msg;
    // =================================================
    // ================= BUFFERS =======================
    // =================================================
    #define DRILL_ARRAY_SIZE 8
    float drill_data_buffer[DRILL_ARRAY_SIZE];

    #define SENSOR_ARRAY_SIZE 14
    float sensor_data_buffer[SENSOR_ARRAY_SIZE];

    int32_t info_warning_buffer;

    int32_t command_data_buffer[5];

    #define CMD_RECEIVED_ARRAY_SIZE 5
    int32_t cmd_received_buffer[CMD_RECEIVED_ARRAY_SIZE];
};

#endif