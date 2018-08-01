#include "longitudinal_controller.hpp"

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "longitudinal_controller_node");

    longitudinal_control_ros_tool::LongitudinalController longitudinal_controller(ros::NodeHandle(), ros::NodeHandle("~"));

    ros::spin();
    return EXIT_SUCCESS;
}
