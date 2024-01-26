#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <mren_interfaces/msg/pico_motor_commands.h>
#include <mren_interfaces/msg/pico_sensor_output.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

rcl_publisher_t publisher;
rcl_subscription_t subscriber;
std_msgs__msg__Int32 msg;
mren_interfaces__msg__PicoMotorCommands motor_commands;
mren_interfaces__msg__PicoSensorOutput sensor_output;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop() {
  while(1) {
    delay(100);
  }
}

// Subscription Callback
void pico_motor_commands_callback(const void *msgin) {
    const mren_interfaces__msg__PicoMotorCommands * msgi = (const mren_interfaces__msg__PicoMotorCommands *)msgin;
    // msg.data = msg->data + 1;
    motor_commands.left_wheel_velocity = msgi->left_wheel_velocity + 1;
    motor_commands.right_wheel_velocity = msgi->right_wheel_velocity + 2;
    sensor_output.left_wheel_velocity = motor_commands.left_wheel_velocity;
    sensor_output.right_wheel_velocity = motor_commands.right_wheel_velocity;
    RCSOFTCHECK(rcl_publish(&publisher, &sensor_output, NULL));
}

// Timer Callback
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &sensor_output, NULL));
  }
}

void setup() {
  // Configure serial transport
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "baby_mario", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(mren_interfaces, msg, PicoSensorOutput),
    "pico_sensor_output"));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
  	&subscriber,
  	&node,
  	ROSIDL_GET_MSG_TYPE_SUPPORT(mren_interfaces, msg, PicoMotorCommands),
  	"pico_motor_commands"));

  // create timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  //RCCHECK(rclc_executor_add_timer(&executor, &timer));

  // Add subscription to the executor
  RCCHECK(rclc_executor_add_subscription(
    &executor, &subscriber, &motor_commands,
    &pico_motor_commands_callback, ON_NEW_DATA));

  msg.data = 0;
  sensor_output.left_wheel_velocity = 0;
  sensor_output.right_wheel_velocity = 0;
  sensor_output.proximity_front = 0;
  sensor_output.proximity_left = 0;
  sensor_output.proximity_right = 0;
  sensor_output.acceleration.ax = 0;
  sensor_output.acceleration.ay = 0;
  sensor_output.acceleration.az = 0;
  sensor_output.angular_rate.rx = 0;
  sensor_output.angular_rate.ry = 0;
  sensor_output.angular_rate.rz = 0;
  motor_commands.left_wheel_velocity = 0;
  motor_commands.right_wheel_velocity = 0;
  RCSOFTCHECK(rclc_executor_spin(&executor));
}

void loop() {
  delay(1000);
  RCSOFTCHECK(rcl_publish(&publisher, &sensor_output, NULL));
  // RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
