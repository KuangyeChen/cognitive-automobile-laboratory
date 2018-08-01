#include "lateral_controller.hpp"

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "lateral_controller_node");

    lateral_control_ros_tool::LateralController lateral_controller(ros::NodeHandle(), ros::NodeHandle("~"));

    ros::spin();
    return EXIT_SUCCESS;
}
