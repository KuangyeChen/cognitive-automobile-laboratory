#include "LandmarkCalibratorInterfaceParameters.h"

#include "../ros_utils.h"

namespace stargazer_ros_tool {

LandmarkCalibratorInterfaceParameters& LandmarkCalibratorInterfaceParameters::getInstance() {
    static LandmarkCalibratorInterfaceParameters p;
    return p;
}

void LandmarkCalibratorInterfaceParameters::fromNodeHandle(const ros::NodeHandle& node_handle) {

    getParam(node_handle, "stargazer_cfg_file_in", stargazer_cfg_file_in);
    getParam(node_handle, "stargazer_cfg_file_out", stargazer_cfg_file_out);
    getParam(node_handle, "bag_file", bag_file);
    getParam(node_handle, "landmark_topic", landmark_topic);
    getParam(node_handle, "pose_topic", pose_topic);
}

LandmarkCalibratorInterfaceParameters::LandmarkCalibratorInterfaceParameters() {
}

} // namespace stargazer_ros_tool
