#include "filter_and_control_tar.hpp"

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "filter_and_control_tar_node");

    kitaf_navigation_ros_tool::FilterAndControlTar filter_and_control_tar(ros::NodeHandle(), ros::NodeHandle("~"));

    ros::spin();
    return EXIT_SUCCESS;
}
