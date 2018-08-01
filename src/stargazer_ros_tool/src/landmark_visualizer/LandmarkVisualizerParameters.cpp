#include "LandmarkVisualizerParameters.h"

#include "../ros_utils.h"

namespace stargazer_ros_tool {

LandmarkVisualizerParameters& LandmarkVisualizerParameters::getInstance() {
    static LandmarkVisualizerParameters p;
    return p;
}

void LandmarkVisualizerParameters::fromNodeHandle(const ros::NodeHandle& node_handle) {

    getParam(node_handle, "stargazer_config", stargazer_config);
    getParam(node_handle, "landmark_topic", landmark_topic);
    getParam(node_handle, "rate", rate);
    getParam(node_handle, "map_frame_id", map_frame_id);
}

LandmarkVisualizerParameters::LandmarkVisualizerParameters() {
}

} // namespace stargazer_ros_tool
