#include <motor_interface/motor_interface_node.hpp>
#include <boost/algorithm/clamp.hpp>
#include "utils_ros/node_handle.hpp"
#include "utils_ros/time_conversions.hpp"

namespace motor_interface_ros_tool {

MotorInterfaceNode::MotorInterfaceNode(ros::NodeHandle nh)
        : gear_ratio(utils_ros::getParam<double>(nh, "gear_ratio")),
          steer_ratio(utils_ros::getParam<double>(nh, "steer_ratio")),
          steer_offset(utils_ros::getParam<double>(nh, "steer_offset")),
          maximal_update_rate(utils_ros::getParam<double>(nh, "maximal_update_rate")),
          is_activated(false),
          velocity(0.0),
          steering(steer_offset),
          motor(utils_ros::getParam<std::string>(nh, "device").c_str()) {
    rossub_motor_command =
        nh.subscribe("motor_command", 5, &MotorInterfaceNode::onMotorCommand, this);
    rossub_servo_command =
        nh.subscribe("servo_command", 5, &MotorInterfaceNode::onServoCommand, this);
    rosserv_activation =
        nh.advertiseService("activate", &MotorInterfaceNode::activateService, this);
    motor_update_timer = 
        nh.createTimer(maximal_update_rate, &MotorInterfaceNode::onMotorUpdate, this);
}

void MotorInterfaceNode::onMotorCommand(const MotorCommand::ConstPtr& motor_command_ptr) {
    if (!is_activated) {
        ROS_WARN_THROTTLE(2, "MotorInterface is inactive!");
        return;
    }

    velocity = motor_command_ptr->velocity / gear_ratio;
}

void MotorInterfaceNode::onServoCommand(const ServoCommand::ConstPtr &servo_command_ptr) {
    if (!is_activated) {
        ROS_WARN_THROTTLE(2, "MotorInterface is inactive!");
        return;
    }

    steering = fromAngle(servo_command_ptr->steering_angle);
}

void MotorInterfaceNode::onMotorUpdate(const ros::TimerEvent& e) {
    ros::Time before_v_set = ros::Time::now();
    const bool success = motor.setVelocity(velocity);
    ros::Time after_v_set = ros::Time::now();

    ROS_DEBUG_STREAM("setVelocity was called at [ms]: "
                     << utils_ros::toMicroSeconds<float>(before_v_set)
                     << "and took "
                     << utils_ros::toMicroSeconds<float>(after_v_set - before_v_set)
                     << "and SetVelocity was "
                     << (success ? "" : "not ")
                     << "successful.");
    if (!success) {
        ROS_ERROR_STREAM("motor controller state: " << motor.getState());
    }

    servo.setSteer(steering);
}

bool MotorInterfaceNode::activateService(Activation::Request& request,
                                         Activation::Response& response) {
    if (request.activate && !is_activated) {
        ROS_INFO("motor_interface has been activated!");
        steering = steer_offset;
        velocity = 0.0;
        is_activated = true;
    } else if (!request.activate && is_activated) {
        ROS_INFO("motor_interface has been deactivated!");
        steering = steer_offset;
        velocity = 0.0;
        is_activated = false;
    }
    return true;
}

unsigned char MotorInterfaceNode::fromAngle(const double angle) {
    const double normalized_angle = std::fmod(angle, M_PI);
    const double ticks = normalized_angle / steer_ratio;
    return MAX_TICKS - static_cast<unsigned char>(boost::algorithm::clamp(
                           std::round(ticks + steer_offset), float(MIN_TICKS), float(MAX_TICKS)));
}
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "motor_interface");
    using Interface = motor_interface_ros_tool::MotorInterfaceNode;
    std::unique_ptr<Interface> node;
    ros::NodeHandle nh("~");
    try {
        ROS_INFO("Start initialization.");
        node = std::make_unique<Interface>(nh);
        ROS_INFO("Finished initialization.");
    } catch (const std::invalid_argument& ex) {
        ROS_ERROR("Could not initialize MotorInterfaceNode: %s", ex.what());
        return -1;
    }

    ros::spin();
    return EXIT_SUCCESS;
}
