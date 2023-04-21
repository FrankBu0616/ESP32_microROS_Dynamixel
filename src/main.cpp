#include "XL330.h"
#include "Arduino.h"
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <HardwareSerial.h>
#include <std_msgs/msg/int32.h>
#include <custom_interfaces/msg/group_command.h>
#include <custom_interfaces/srv/change_mode.h>
#include <micro_ros_platformio.h>

// ESP32 Pinout
#define ledPin      2
#define RXD2        16
#define TXD2        17
#define velControl  1
#define posControl  3

// micro-ros setup
rcl_publisher_t position_publisher;
rcl_subscription_t command_subscriber;
std_msgs__msg__Int32 position_msg;
custom_interfaces__msg__GroupCommand command_msg;
custom_interfaces__srv__ChangeMode_Request server_req;
custom_interfaces__srv__ChangeMode_Response server_res;

const rosidl_message_type_support_t * type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);
const rosidl_message_type_support_t * custom_support = ROSIDL_GET_MSG_TYPE_SUPPORT(custom_interfaces, msg, GroupCommand);

rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;
rcl_service_t service;

unsigned int num_handles = 3;   // 1 subscriber, 1 publisher, 1 server
// Initialize dynamixel interace.
XL330 xl330;
// Dynamixel Motor setup
const int num_joints = 2;
int servoIDs[num_joints] = {1, 2};
int controlMode = velControl;
HardwareSerial motorSerial(2);  // using UART2

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); return;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}
#define RUNWAIT(fn) {fn; delay(10);}
#define RUNWAIT_LONG(fn) {fn; delay(100);}

// Error handle loop
void error_loop() {
  while(1) {
    delay(100);
  }
}

void blink_esp32_led() {
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);
  delay(300);
  digitalWrite(ledPin, LOW);
}

void blink_motor_leds() {
  for (int i = 0; i < num_joints; i++) {
    RUNWAIT(xl330.LEDON(servoIDs[i]));
  }
  delay(300);
  for (int i = 0; i < num_joints; i++) {
    RUNWAIT(xl330.LEDOFF(servoIDs[i]));
  }
}

void changeMotorMode(int mode) {
  for (int i = 0; i < num_joints; i++) {
    RUNWAIT_LONG(xl330.TorqueOFF(servoIDs[i]));
    RUNWAIT_LONG(xl330.setControlMode(servoIDs[i], mode)); // Velocity Control
    RUNWAIT_LONG(xl330.enableMaxVelocity(servoIDs[i]));
    RUNWAIT_LONG(xl330.TorqueON(servoIDs[i]));
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
  const custom_interfaces__msg__GroupCommand * command_msg = (const custom_interfaces__msg__GroupCommand *) msgin;  

  // Process message
  if (controlMode == velControl) {
    for (int i = 0; i < num_joints; i++) {
      RUNWAIT(xl330.setJointSpeed(servoIDs[i], command_msg->values.data[i]));
    }
  }

  if (controlMode == posControl) {
    for (int i = 0; i < num_joints; i++) {
      RUNWAIT(xl330.setJointPosition(servoIDs[i], command_msg->values.data[i]));
    }
  }
}

void service_callback(const void * req, void * res){
  custom_interfaces__srv__ChangeMode_Request * req_in = (custom_interfaces__srv__ChangeMode_Request *) req;
  custom_interfaces__srv__ChangeMode_Response * res_in = (custom_interfaces__srv__ChangeMode_Response *) res;
  changeMotorMode(req_in->mode);
  controlMode = req_in->mode;
  res_in->confirmation = 1;
  blink_esp32_led();
  for (int i = 0; i < 2; i++) RUNWAIT_LONG(blink_motor_leds());
}


void setup() {
  // Open Serial for monitoring
  // Serial.begin(115200);
  // RUNWAIT_LONG(set_microros_serial_transports(Serial));

  // WiFi settings
  IPAddress agent_ip(192, 168, 1, 2);
  size_t agent_port = 8888;

  char ssid[] = "wifi";
  char psk[]= "password";
  set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);

  // Setup and initiate Software Serial for robot
  motorSerial.begin(115200, SERIAL_8N1, RXD2, TXD2);
  xl330.begin(motorSerial); // Hand in the serial object you're using

  changeMotorMode(1);

  // allocate space for customized message
  command_msg.values.capacity = 2;
  command_msg.values.data = (int32_t*) malloc(command_msg.values.capacity * sizeof(int32_t));
  command_msg.values.size = 0;
  
  allocator = rcl_get_default_allocator();

  // Init node
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "dbot_node", "", &support));

  // create publisher and subscriber
  RCCHECK(rclc_publisher_init_default(&position_publisher, &node, type_support, "dynamixel_position_publisher"));
  RCCHECK(rclc_subscription_init_default(&command_subscriber, &node, custom_support, "goal_command"));

  // create service
  RCCHECK(rclc_service_init_default(&service, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(custom_interfaces, srv, ChangeMode), "/changemode"));

  // create timer,
  const unsigned int timer_timeout = 10;
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback));

  // create executor, one for publisher and one for subscriber.
  RCCHECK(rclc_executor_init(&executor, &support.context, num_handles, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &command_subscriber, &command_msg, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_service(&executor, &service, &server_req, &server_res, service_callback));

  // Blink LED once the server is up
  blink_esp32_led();
  blink_motor_leds();
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}

