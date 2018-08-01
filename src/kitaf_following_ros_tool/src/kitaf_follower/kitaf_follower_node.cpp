#include "kitaf_follower.hpp"

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "kitaf_follower_node");

    kitaf_following_ros_tool::KitafFollower kitaf_follower(ros::NodeHandle(), ros::NodeHandle("~"));

    ros::spin();
    return EXIT_SUCCESS;
}
