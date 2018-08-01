#pragma once

#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>

#include <sensor_msgs/Image.h>

#include "image_preproc_ros_tool/LabelDilationInterface.h"

namespace image_preproc_ros_tool {

class LabelDilation {

    using Interface = LabelDilationInterface;
    using ReconfigureConfig = LabelDilationConfig;
    using ReconfigureServer = dynamic_reconfigure::Server<ReconfigureConfig>;

    using Msg = sensor_msgs::Image;

public:
    LabelDilation(ros::NodeHandle, ros::NodeHandle);

private:
    void callbackSubscriber(const Msg::ConstPtr& msg);
    void reconfigureRequest(const ReconfigureConfig&, uint32_t);

    Interface interface_;
    ReconfigureServer reconfigureServer_;
};
} // namespace image_preproc_ros_tool
