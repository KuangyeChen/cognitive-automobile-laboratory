#pragma once

#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <stargazer/DebugVisualizer.h>
#include <stargazer/LandmarkFinder.h>
#include <stargazer_ros_tool/LandmarkFinderConfig.h>

#include "LandmarkFinderInterfaceParameters.h"

namespace stargazer_ros_tool {

class LandmarkFinderInterface {

public:
    LandmarkFinderInterface(ros::NodeHandle = ros::NodeHandle(),
                            ros::NodeHandle = ros::NodeHandle("~"));

private:
    void imgCallback(const sensor_msgs::ImageConstPtr& msg);
    void reconfigureCallback(LandmarkFinderConfig& config, const uint32_t& level = 0);

    image_transport::Subscriber img_sub;
    image_transport::ImageTransport img_trans;
    ros::Publisher lm_pub;
    dynamic_reconfigure::Server<LandmarkFinderConfig> server;
    LandmarkFinderInterfaceParameters params_;
    stargazer::DebugVisualizer debugVisualizer_;
    std::unique_ptr<stargazer::LandmarkFinder> landmarkFinder;
};
}
