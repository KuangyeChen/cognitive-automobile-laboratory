#pragma once

#include <string>
#include <ros/node_handle.h>

namespace stargazer_ros_tool {

struct LandmarkLocalizerInterfaceParameters {

    void fromNodeHandle(const ros::NodeHandle&);

    std::string stargazer_config;
    std::string map_frame;
    std::string camera_frame;
    std::string landmark_topic;
    std::string pose_topic;
    bool debug_mode;
    bool estimate_2d_pose;
};
}
