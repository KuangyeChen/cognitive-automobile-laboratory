#include "Joy2MotorCommandConverter.h"

#include "utils_ros/node_handle.hpp"

#include "motor_interface_ros_tool/MotorCommand.h"
#include "motor_interface_ros_tool/ServoCommand.h"
#include "motor_interface_ros_tool/Activation.h"

namespace joy_converter_ros_tool {

Joy2MotorCommandConverter::Joy2MotorCommandConverter(ros::NodeHandle &node_handle) :
  velocity_joystick_axis(utils_ros::getParam<unsigned int>(node_handle, "velocity_joystick_axis")),
  steering_joystick_axis(utils_ros::getParam<unsigned int>(node_handle, "steering_joystick_axis")),
  activate_joystick_button(utils_ros::getParam<unsigned int>(node_handle, "activation_joystick_button")),
  deactivate_joystick_button(utils_ros::getParam<unsigned int>(node_handle, "deactivation_joystick_button")),
  max_velocity(utils_ros::getParam<double>(node_handle, "max_velocity")),
  max_steering_angle(utils_ros::getParam<double>(node_handle, "max_steering_angle"))
{
  rossub_joy = node_handle.subscribe("joy", 5, &Joy2MotorCommandConverter::onJoyUpdate, this);
  rospub_motor_command = node_handle.advertise<motor_interface_ros_tool::MotorCommand>("motor_command", 5);
  rospub_servo_command = node_handle.advertise<motor_interface_ros_tool::ServoCommand>("servo_command", 5);
  rosclient_activation = node_handle.serviceClient<motor_interface_ros_tool::Activation>("activate");
}

void Joy2MotorCommandConverter::onJoyUpdate(const sensor_msgs::JoyConstPtr &joy_msg)
{
  // Ensure that not both buttons are pressed at the same time
  if (joy_msg->buttons[activate_joystick_button] && !joy_msg->buttons[deactivate_joystick_button]) {
    activateMotorInterface(true);
  } else if (joy_msg->buttons[deactivate_joystick_button] && !joy_msg->buttons[activate_joystick_button]) {
    activateMotorInterface(false);
  }

  motor_interface_ros_tool::MotorCommand motor_command;
  motor_command.header.stamp = joy_msg->header.stamp;
  motor_command.velocity = max_velocity * joy_msg->axes[velocity_joystick_axis];
  rospub_motor_command.publish(motor_command);

    const double steering_angle = max_steering_angle * joy_msg->axes[steering_joystick_axis];
  if (std::abs(steering_angle - last_steering_angle) > 0.0001) {
    last_steering_angle = steering_angle;

    motor_interface_ros_tool::ServoCommand servo_command;
    servo_command.header.stamp = joy_msg->header.stamp;
    servo_command.steering_angle = steering_angle;
    rospub_servo_command.publish(servo_command);
  }
}

void Joy2MotorCommandConverter::activateMotorInterface(const bool activate)
{
  motor_interface_ros_tool::Activation activation_call;
  activation_call.request.activate = activate;
  rosclient_activation.call(activation_call);
}

}

