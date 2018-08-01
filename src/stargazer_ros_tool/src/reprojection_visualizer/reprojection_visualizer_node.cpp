#include "ReprojectionVisualizer.h"

int main(int argc, char** argv) {

    ros::init(argc, argv, "reprojection_visualizer_node");

    stargazer_ros_tool::ReprojectionVisualizer reprojectionVisualizer(ros::NodeHandle(), ros::NodeHandle("~"));

    //    ros::spin(); // We don't need that here
    return EXIT_SUCCESS;
}
