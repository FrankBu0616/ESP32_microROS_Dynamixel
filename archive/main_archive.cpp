#include "XL330.h"
#include "Arduino.h"
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <HardwareSerial.h>
#include <std_msgs/msg/int32.h>
#include <custom_msgs/msg/group_command.h>
#include <micro_ros_platformio.h>
// #include <micro_ros_utilities/type_utilities.h>
// #include <micro_ros_utilities/string_utilities.h>
// #include <micro_ros_utilities/visibility_control.h>


// ESP32 Pinout
#define ledPin      2
#define RXD2        16
#define TXD2        17
#define WAIST       1
#define SHOULDER    2

// micro-ros setup
rcl_publisher_t position_publisher;
rcl_subscription_t command_subscriber;
std_msgs__msg__Int32 position_msg;
custom_msgs__msg__GroupCommand command_msg;

const rosidl_message_type_support_t * type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);
const rosidl_message_type_support_t * custom_support = ROSIDL_GET_MSG_TYPE_SUPPORT(custom_msgs, msg, GroupCommand);

rclc_executor_t pub_executor;
rclc_executor_t sub_executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;

// Initialize dynamixel interace.
XL330 xl330;
// Dynamixel Motor setup
const int numOfJoints = 2;
int servoIDs[numOfJoints] = {1, 2};
int servo1Speed = 0;
HardwareSerial motorSerial(2);  // using UART2

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#define RUNWAIT(fn) {fn; delay(100);}
#define RUNWAIT_LONG(fn) {fn; delay(2000);}

// Error handle loop
void error_loop() {
  while(1) {
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  position_msg.data = xl330.getJointPosition(servoIDs[0]);
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&position_publisher, &position_msg, NULL));
  }
}

void subscription_callback(const void * msgin)
{
  // Cast received message to used type
  position_msg.data = 6;
  const custom_msgs__msg__GroupCommand * command_msg = (const custom_msgs__msg__GroupCommand *)msgin;
  RCSOFTCHECK(rcl_publish(&position_publisher, &position_msg, NULL));

  // Process message
  // for (int i = 0; i < numOfJoints; i++) {
  //   RUNWAIT(xl330.setJointPosition(servoIDs[i], (int)command_msg->values.data[i]));
  // }
}


void setup() {
  // Open Serial for monitoring
  // Serial.begin(115200);
  // while(!Serial);
  // RUNWAIT_LONG(set_microros_serial_transports(Serial));
  IPAddress agent_ip(192, 168, 1, 22);
  size_t agent_port = 8888;

  char ssid[] = "KittyCatter"; // "Enoki";
  char psk[]= "FluffyLink"; // "W0nton$oup";
  set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);

  // char ssid[] = "Enoki";
  // char psk[]= "W0nton$oup";
  // set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);

  // Setup and initiate Software Serial for robot
  motorSerial.begin(115200, SERIAL_8N1, RXD2, TXD2);
  xl330.begin(motorSerial); // Hand in the serial object you're using

  for (int i = 0; i < numOfJoints; i++) {
    RUNWAIT(xl330.TorqueOFF(servoIDs[i]));
    RUNWAIT(xl330.setControlMode(servoIDs[i], 3)); // Position Control
    RUNWAIT(xl330.TorqueON(servoIDs[i]));
  }

  static int32_t memory[2];
  command_msg.values.capacity = 2;
  command_msg.values.data = memory;
  command_msg.values.size = 0;
  

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "dynamixel_node", "", &support));

  // create publisher and subscriber
  RCCHECK(rclc_publisher_init_default(&position_publisher, &node, type_support, "dynamixel_position_publisher"));
  RCCHECK(rclc_subscription_init_default(&command_subscriber, &node, custom_support, "goal_position"));

  Serial.println("initialize ROS");
  // create timer,
  const unsigned int timer_timeout = 10;
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback));

  // create executor, one for publisher and one for subscriber.
  RCCHECK(rclc_executor_init(&pub_executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&pub_executor, &timer));
  RCCHECK(rclc_executor_init(&sub_executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&sub_executor, &command_subscriber, &command_msg, &subscription_callback, ON_NEW_DATA));

  // Blink LED once the server is up
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);
  delay(300);
  digitalWrite(ledPin, LOW);
  
  for (int i = 0; i < numOfJoints; i++) {
    RUNWAIT(xl330.LEDON(servoIDs[i]));
  }
}

void loop() {
  Serial.println("test");
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&pub_executor, RCL_MS_TO_NS(20)));
  RCSOFTCHECK(rclc_executor_spin_some(&sub_executor, RCL_MS_TO_NS(20)));
}