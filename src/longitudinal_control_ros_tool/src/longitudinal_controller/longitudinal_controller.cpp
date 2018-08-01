#include "longitudinal_controller.hpp"
#include <utils_ros/ros_console.hpp>

#include "motor_interface_ros_tool/MotorCommand.h"

namespace longitudinal_control_ros_tool {

using MotorCommand = motor_interface_ros_tool::MotorCommand;

LongitudinalController::LongitudinalController(ros::NodeHandle nh_public, ros::NodeHandle nh_private)
        : params_{nh_private}, reconfigureServer_{nh_private} {

    /**
     * Initialization
     */
    utils_ros::setLoggerLevel(nh_private);
    params_.fromParamServer();

    /**
     * Publishers & subscriber
     */

    motor_command_publisher_ = nh_private.advertise<MotorCommand>(params_.motor_command_topic, params_.msg_queue_size);

    // Instantiate subscriber last, to assure all objects are initialised when first message is received.
    control_loop_timer_ =
        nh_private.createTimer(ros::Rate(params_.control_loop_rate), &LongitudinalController::controlLoopCallback, this);

    /**
     * Set up dynamic reconfiguration
     */
    reconfigureServer_.setCallback(boost::bind(&LongitudinalController::reconfigureRequest, this, _1, _2));

    utils_ros::showNodeInfo();
}

void LongitudinalController::controlLoopCallback(const ros::TimerEvent& timer_event) {

    MotorCommand motor_command;
    motor_command.header.stamp = timer_event.current_expected;
    motor_command.velocity = params_.velocity;

    motor_command_publisher_.publish(motor_command);
}

/**
  * This callback is called at startup or whenever a change was made in the dynamic_reconfigure window
*/
void LongitudinalController::reconfigureRequest(const Config& config, uint32_t level) {
    params_.fromConfig(config);
}


} // namespace longitudinal_control_ros_tool
