#pragma once

#include <dynamic_reconfigure/server.h>
#include <ros/forwards.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Eigen>
#include <nav_msgs/Path.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include <kitaf_utils_ros/MotorState.h>
#include <kitaf_utils_ros/detection_signals.hpp>
#include <kitaf_utils_ros/SwitchToExit.h>

#include "kitaf_lateral_control_ros_tool/KitafControllerInterface.h"


namespace kitaf_lateral_control_ros_tool {

class KitafController {

    using Interface = KitafControllerInterface;
    using ReconfigureConfig = KitafControllerConfig;
    using ReconfigureServer = dynamic_reconfigure::Server<ReconfigureConfig>;

public:
    KitafController(ros::NodeHandle, ros::NodeHandle);

private:
    void reconfigureRequest(const ReconfigureConfig&, uint32_t);
    void controlLoopCallback(const ros::TimerEvent& timer_event);
    void velocityLoopCallback(const ros::TimerEvent& );
    void pathCallback(const nav_msgs::Path::ConstPtr& msg);
    void StopSignCallback(const std_msgs::Float64::ConstPtr& msg);
    void motorStateCallback(const kitaf_utils_ros::MotorState::ConstPtr& msg);
    void currentTaskCallback(const std_msgs::Int8::ConstPtr& msg);
    std::vector<Eigen::Affine3d> path_;

    ros::Subscriber path_subscriber_, stopSign_subcriber_, motor_state_subscriber_, currentTask_subscriber_;
    ros::Publisher servo_command_publisher_, motor_command_publisher_, debug_publisher_;
    ros::Timer control_loop_timer_, velocity_loop_timer_;


    Interface interface_;
    ReconfigureServer reconfigureServer_;

    /**
     * Machine state: [current velocity, current steer angle]
     */
    Eigen::Vector2d motor_state_;
    double StopSignal;
    bool near_center, pass_center, ready_watch, exit_state;
    double lastSteeringAngle;
    double lastYawAngle;
    double k_velocity;

    int currentTask;
    double k_yaw, k_yaw_error, k_dis, velocity;
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    tf2_ros::TransformBroadcaster tfBroadcaster_;
};
} // namespace kitaf_lateral_control_ros_tool
