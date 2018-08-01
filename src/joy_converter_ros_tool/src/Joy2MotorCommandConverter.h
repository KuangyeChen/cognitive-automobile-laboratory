#ifndef JOY2MOTORCOMMANDCONVERTER_H
#define JOY2MOTORCOMMANDCONVERTER_H

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

namespace joy_converter_ros_tool {

class Joy2MotorCommandConverter
{
public:
  Joy2MotorCommandConverter(ros::NodeHandle& node_handle);

  void onJoyUpdate(const sensor_msgs::JoyConstPtr& joy_msg);

private:
  void activateMotorInterface(const bool activate);

  const unsigned int velocity_joystick_axis;
  const unsigned int steering_joystick_axis;
  const unsigned int activate_joystick_button;
  const unsigned int deactivate_joystick_button;
  const double max_velocity;
  const double max_steering_angle;

  double last_steering_angle = 0.0;

  ros::Subscriber rossub_joy;
  ros::Publisher rospub_motor_command;
  ros::Publisher rospub_servo_command;
  ros::ServiceClient rosclient_activation;
};

} // namespace joy_converter_ros_tool

#endif // JOY2MOTORCOMMANDCONVERTER_H
