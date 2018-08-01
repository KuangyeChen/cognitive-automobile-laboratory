#pragma once

#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/Path.h>

#include <Eigen/Dense>

#include "lateral_control_ros_tool/LateralControllerParameters.h"

namespace lateral_control_ros_tool {

class LateralController {

    using Parameters = LateralControllerParameters;
    using Config = LateralControllerConfig;
    using ReconfigureServer = dynamic_reconfigure::Server<Config>;

public:
    LateralController(ros::NodeHandle, ros::NodeHandle);

private:
    void pathCallback(const nav_msgs::Path::ConstPtr& msg);
    void controlLoopCallback(const ros::TimerEvent& timer_event);
    void reconfigureRequest(const Config&, uint32_t);

    std::vector<Eigen::Affine3d> path_;

    ros::Publisher servo_command_publisher_;
    ros::Subscriber path_subscriber_;
    ros::Timer control_loop_timer_;
    Parameters params_;
    ReconfigureServer reconfigureServer_;

    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    tf2_ros::TransformBroadcaster tfBroadcaster_;
};
} // namespace lateral_control_ros_tool
