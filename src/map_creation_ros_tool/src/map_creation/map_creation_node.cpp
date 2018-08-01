#include "map_creation.hpp"

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "map_creation_node");

    map_creation_ros_tool::MapCreation map_creation(ros::NodeHandle(), ros::NodeHandle("~"));

    ros::spin();
    return EXIT_SUCCESS;
}
