#pragma once

#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <std_msgs/Header.h>

#include "longitudinal_control_ros_tool/LongitudinalControllerParameters.h"

namespace longitudinal_control_ros_tool {

class LongitudinalController {

    using Parameters = LongitudinalControllerParameters;
    using Config = LongitudinalControllerConfig;
    using ReconfigureServer = dynamic_reconfigure::Server<Config>;

public:
    LongitudinalController(ros::NodeHandle, ros::NodeHandle);

private:
    void controlLoopCallback(const ros::TimerEvent& timer_event);
    void reconfigureRequest(const Config&, uint32_t);

    ros::Publisher motor_command_publisher_;
    ros::Timer control_loop_timer_;
    Parameters params_;
    ReconfigureServer reconfigureServer_;
};
} // namespace longitudinal_control_ros_tool
