#pragma once

#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include "motor_interface_ros_tool/MotorCommand.h"
#include "motor_interface_ros_tool/ServoCommand.h"

#include "vehicle_model.h"
#include "simulation_ros_tool/VehicleSimulatorParameters.h"

namespace simulation_ros_tool {

class VehicleSimulator {

    using Parameters = VehicleSimulatorParameters;
    using Config = VehicleSimulatorConfig;
    using ReconfigureServer = dynamic_reconfigure::Server<Config>;

public:
    VehicleSimulator(ros::NodeHandle, ros::NodeHandle);

private:
    void onMotorCommand(const motor_interface_ros_tool::MotorCommandConstPtr& msg);
    void onServoCommand(const motor_interface_ros_tool::ServoCommandConstPtr& msg);
    void onTfTimerEvent(const ros::TimerEvent& timer_event);
    void reconfigureRequest(const Config&, uint32_t);

    ros::Subscriber motor_command_subscriber_;
    ros::Subscriber servo_command_subscriber_;
    ros::Timer tf_timer;
    Parameters params_;
    ReconfigureServer reconfigureServer_;

    VehicleModel vehicle_model_;

    tf2_ros::TransformBroadcaster tfBroadcaster_;
};
} // namespace simulation_ros_tool
