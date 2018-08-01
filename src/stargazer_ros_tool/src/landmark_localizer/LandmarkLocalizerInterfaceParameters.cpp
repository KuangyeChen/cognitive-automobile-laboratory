#include "LandmarkLocalizerInterfaceParameters.h"

#include "../ros_utils.h"

namespace stargazer_ros_tool {

void LandmarkLocalizerInterfaceParameters::fromNodeHandle(const ros::NodeHandle& node_handle) {

    getParam(node_handle, "stargazer_config", stargazer_config);
    getParam(node_handle, "landmark_topic", landmark_topic);
    getParam(node_handle, "pose_topic", pose_topic);
    getParam(node_handle, "map_frame", map_frame);
    getParam(node_handle, "camera_frame", camera_frame);
    getParam(node_handle, "debug_mode", debug_mode);
    getParam(node_handle, "estimate_2d_pose", estimate_2d_pose);
}
}
