# ROS Motor Interface 

This package contains the node `motor_interface` which provides a ROS-Interface to communicate with the motors on the anicars.

## Installation

Look at [motor_interface](https://gitlab.mrt.uni-karlsruhe.de/kognitive_automobile_labor/motor_interface) for install instructions.

## Usage

Launch: `roslaunch motor_interface_ros_tool motor_interface_node.launch`. The
`motor_interface`-node will start deactivated. To activate it, call `activation`-service.

## Nodes
### motor_interface_ros_tool

#### Parameter
  * `device` (string): the serial device to communicate with the EPOS-Motor-driver.
  * `feedback_rate`(double): rate to publish the feedback (current steering angle and motor velocity) with [hz].
  * `gear_ratio`(double): ratio between vehicle velocity and motor rotation rate [(m/s)/rpm].
  * `steer_ratio`(double): ratio between servo-ticks and steering angle [ticks/rad].
  * `steer_offset`(int): zero steering angle position of servo [ticks].

### Subscriptions
  * `motor_command`(motor_interface_ros_tool/MotorCommand): motor-commands to execute
  * `servo_command`(motor_interface_ros_tool/ServoCommand): servo-commands to execute

**Note:** This node is designed to be replaceable by [vehicle_simulator](https://gitlab.mrt.uni-karlsruhe.de/kognitive_automobile_labor/simulation_ros_tool). Therefore these topics should not be changed here!

### Service
  * `activate`(motor_interface_ros_tool/Activation): activates/deactivates motor interface.

## Contributing

1. Fork it!
2. Create your feature branch: `git checkout -b my-new-feature`
3. Commit your changes: `git commit -am 'Add some feature'`
4. Push to the branch: `git push origin my-new-feature`
5. Submit a pull request :D
