//
// Created by bandera on 10.06.16.
//

#pragma once

#include <memory>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include "ReprojectionVisualizerParameters.h"
#include "stargazer/DebugVisualizer.h"
#include "stargazer_ros_tool/LandmarkArray.h"

namespace stargazer_ros_tool {

class ReprojectionVisualizer {
public:
    ReprojectionVisualizer(ros::NodeHandle, ros::NodeHandle);

private:
    ReprojectionVisualizerParameters& params_;
    stargazer::landmark_map_t landmarks;
    stargazer::camera_params_t camera_intrinsics = {{0., 0., 0., 0.}};

    std::unique_ptr<stargazer::DebugVisualizer> debugVisualizer_;

    void synchronizerCallback(const stargazer_ros_tool::LandmarkArray::ConstPtr& lm_msg,
                              const geometry_msgs::PoseStamped::ConstPtr& pose_msg,
                              const sensor_msgs::ImageConstPtr& img_msg);
};

} // namespace stargazer_ros_tool
