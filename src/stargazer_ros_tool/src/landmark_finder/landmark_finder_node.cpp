#include "LandmarkFinderInterface.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "landmark_finder_node");
    stargazer_ros_tool::LandmarkFinderInterface interface;
    ros::spin();
    return EXIT_SUCCESS;
}
