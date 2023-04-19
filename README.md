# micro_movement

Template to build customized robot with Dynamixel 330 motors and ESP32. Motor control is managed by micro-ROS. Therefore, heavy computation can be off-load elsewhere. The goal of this repo is to provide a template to control small robots (power by micro-controllers) with ROS 2.

The Dynamixle Motor interface is based on Dr. Rei Lee's [Dynamixel_XL330_Servo_Library](https://github.com/rei039474/Dynamixel_XL330_Servo_Library).

To run the template, PlatformIO IDE is required.

## Hardware

- Dynamixel 330 (x 2)
- ESP32-dev board
- battery 
- jumper wires
- material of your choice for the chassis
- a laptop running linux

## Software
- PlatformIO (VS Code IDE)
- micro-ROS
- ROS 2 Humble

## Setup
To setup the motors, you can either use a U2D2 and the wizard app, or you can use an esp32 with the example script located in `lib/XL330/examples/xl330_baud_rate_and_id`. 
- Motor 1
    - ID: 1
    - baudrate: 115200
- Motor 2
    - ID: 2
    - baudrate: 115200

I am using Serial2 to communicate, so all you need to do is connect motor1's data pin to `TX2` and `RX2` on the board.

## Communication
- wifi

- Serial

- micro-ros agent

## Code Structure
- customized ROS packages, such as messages and services, should go under `/extra_packacges`. If you make changes to these packages, you must rebuild ROS.
- external libraries should live under `/lib` following the structure specified in `/lib/README`
- `main.cpp` contains the actual execution. `setup()` run only once and should be used to initialize ROS nodes. `loop()` function should spin the executors.

### Topics
There are two available topics in this example,
-`dynamixel_position_publisher` publishing the joint position of the first motor.
-`goal_command` is used to receive goal positions/velocities.

### Services
- `changemode` ROS service is available to change control modes. Right now, the code only supports position control and velocity control. But you can easily modify the code to adapt to other modes.