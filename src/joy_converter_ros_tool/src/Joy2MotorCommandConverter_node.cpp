#include "Joy2MotorCommandConverter.h"

int main (int argc, char* argv[]) {
  ros::init(argc, argv, "joy2motor");

  ros::NodeHandle node_handle("~");
  joy_converter_ros_tool::Joy2MotorCommandConverter Joy2MotorCommandConverter(node_handle);
  ros::spin();
  return EXIT_SUCCESS;
}
