#include "kitaf_path_provider.hpp"

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "kitaf_path_provider_node");

    kitaf_navigation_ros_tool::KitafPathProvider kitaf_path_provider(ros::NodeHandle(), ros::NodeHandle("~"));

    ros::spin();
    return EXIT_SUCCESS;
}
