#pragma once

#include <dynamic_reconfigure/server.h>
#include <ros/forwards.h>
#include <nav_msgs/Path.h>
#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "kitaf_following_ros_tool/KitafFollowerInterface.h"

namespace kitaf_following_ros_tool {

class KitafFollower {

    using Interface = KitafFollowerInterface;
    using ReconfigureConfig = KitafFollowerConfig;
    using ReconfigureServer = dynamic_reconfigure::Server<ReconfigureConfig>;

    using Msg = std_msgs::Header;

public:
    KitafFollower(ros::NodeHandle, ros::NodeHandle);

private:
    void callbackSubscriber(const Msg::ConstPtr& msg);
    void reconfigureRequest(const ReconfigureConfig&, uint32_t);
    void controlLoopCallback(const ros::TimerEvent& timer_event);
    void targetPoseCallback(const geometry_msgs::PoseStamped::ConstPtr & msg);

    Interface interface_;
    ReconfigureServer reconfigureServer_;

    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    tf2_ros::TransformBroadcaster tfBroadcaster_;

    ros::Subscriber targetPose_subscriber_;
    ros::Publisher servo_command_publisher_, motor_command_publisher_;
    ros::Timer control_loop_timer_;
    std::vector<Eigen::Affine3d> path_;
    Eigen::Vector3d pose_;
    bool posecall;
};
} // namespace kitaf_following_ros_tool
