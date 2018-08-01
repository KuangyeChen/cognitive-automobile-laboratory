#include "LandmarkLocalizerInterface.h"

int main(int argc, char** argv) {

    ros::init(argc, argv, "landmark_localizer_node");

    stargazer_ros_tool::LandmarkLocalizerInterface interface(ros::NodeHandle(),
                                                             ros::NodeHandle("~"));

    ros::spin();
    return EXIT_SUCCESS;
}
