#pragma once

#include <string>
#include <ros/node_handle.h>

namespace stargazer_ros_tool {

struct LandmarkVisualizerParameters {

    static LandmarkVisualizerParameters& getInstance();

    void fromNodeHandle(const ros::NodeHandle&);

    std::string stargazer_config;
    std::string landmark_topic;
    std::string map_frame_id;
    double rate;

private:
    LandmarkVisualizerParameters();
};

} // namespace stargazer_ros_tool
