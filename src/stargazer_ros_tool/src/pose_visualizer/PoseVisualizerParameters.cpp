#include "PoseVisualizerParameters.h"

#include "../ros_utils.h"

namespace stargazer_ros_tool {

PoseVisualizerParameters& PoseVisualizerParameters::getInstance() {
    static PoseVisualizerParameters p;
    return p;
}

void PoseVisualizerParameters::fromNodeHandle(const ros::NodeHandle& node_handle) {

    getParam(node_handle, "bag_file", bag_file);
    getParam(node_handle, "stargazer_config", stargazer_config);
    getParam(node_handle, "map_frame", map_frame);
    getParam(node_handle, "camera_frame", camera_frame);
    getParam(node_handle, "landmark_topic", landmark_topic);
    getParam(node_handle, "pose_pub_topic", pose_pub_topic);
    getParam(node_handle, "rate", rate);
}

PoseVisualizerParameters::PoseVisualizerParameters() {
}

} // namespace stargazer_ros_tool
