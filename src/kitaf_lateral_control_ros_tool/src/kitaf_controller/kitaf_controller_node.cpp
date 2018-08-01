#include "kitaf_controller.hpp"

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "kitaf_controller_node");

    kitaf_lateral_control_ros_tool::KitafController kitaf_controller(ros::NodeHandle(), ros::NodeHandle("~"));

    ros::spin();
    return EXIT_SUCCESS;
}
