#include "path_provider.hpp"

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "path_provider_node");
    path_provider_ros_tool::PathProvider path_provider;
    ros::spin();
    return EXIT_SUCCESS;
}
