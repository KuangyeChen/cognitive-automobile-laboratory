#include "pylon_detector.hpp"

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "pylon_detector_node");
    demo_image_processing_ros_tool::PylonDetector pylon_detector;
    ros::spin();
    return EXIT_SUCCESS;
}
