#include "path_planning.hpp"

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "path_planning_node");

    kitaf_navigation_ros_tool::PathPlanning path_planning(ros::NodeHandle(), ros::NodeHandle("~"));

    ros::spin();
    return EXIT_SUCCESS;
}
