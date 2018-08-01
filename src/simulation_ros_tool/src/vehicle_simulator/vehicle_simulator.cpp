#include "vehicle_simulator.hpp"
#include <tf2_eigen/tf2_eigen.h>

#include <utils_ros/ros_console.hpp>

namespace simulation_ros_tool {

using namespace Eigen;

VehicleSimulator::VehicleSimulator(ros::NodeHandle nh_public, ros::NodeHandle nh_private)
        : params_{nh_private}, reconfigureServer_{nh_private} {

    utils_ros::setLoggerLevel(nh_private);
    params_.fromParamServer();

    vehicle_model_ = VehicleModel(Translation3d(params_.initial_x, params_.initial_y, 0) *
                                  AngleAxisd(params_.initial_yaw, Vector3d::UnitZ()));

    vehicle_model_.setWheelBase(params_.wheel_base);

    motor_command_subscriber_ = nh_private.subscribe(params_.motor_command_topic,
                                                     params_.msg_queue_size,
                                                     &VehicleSimulator::onMotorCommand,
                                                     this,
                                                     ros::TransportHints().tcpNoDelay());
    servo_command_subscriber_ = nh_private.subscribe(params_.servo_command_topic,
                                                     params_.msg_queue_size,
                                                     &VehicleSimulator::onServoCommand,
                                                     this,
                                                     ros::TransportHints().tcpNoDelay());
    tf_timer = nh_private.createTimer(ros::Rate(params_.tf_refresh_rate), &VehicleSimulator::onTfTimerEvent, this);

    reconfigureServer_.setCallback(boost::bind(&VehicleSimulator::reconfigureRequest, this, _1, _2));

    utils_ros::showNodeInfo();
}

void VehicleSimulator::onMotorCommand(const motor_interface_ros_tool::MotorCommandConstPtr &msg) {
    vehicle_model_.setVelocity(msg->velocity);
}

void VehicleSimulator::onServoCommand(const motor_interface_ros_tool::ServoCommandConstPtr &msg) {
    vehicle_model_.setSteeringAngle(msg->steering_angle);
}

void VehicleSimulator::onTfTimerEvent(const ros::TimerEvent& timer_event) {
    const double time_step = (timer_event.current_real - timer_event.last_real).toSec();
    geometry_msgs::TransformStamped transform = tf2::eigenToTransform(vehicle_model_.step(time_step).inverse());
    transform.child_frame_id = params_.map_frame;
    transform.header.frame_id = params_.vehicle_frame;
    transform.header.stamp = timer_event.current_real;
    tfBroadcaster_.sendTransform(transform);
}

/**
  * This callback is called at startup or whenever a change was made in the dynamic_reconfigure window
*/
void VehicleSimulator::reconfigureRequest(const Config& config, uint32_t level) {
    params_.fromConfig(config);
    vehicle_model_.setWheelBase(params_.wheel_base);
}


} // namespace simulation_ros_tool
