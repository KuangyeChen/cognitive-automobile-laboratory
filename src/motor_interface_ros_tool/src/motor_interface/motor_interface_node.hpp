#ifndef MOTORINTERFACENODE_H
#define MOTORINTERFACENODE_H

#include <motor_interface/EPOS.h>
#include <motor_interface/servo.h>
#include <ros/ros.h>
#include <ros/service_server.h>

#include <motor_interface_ros_tool/Activation.h>
#include <motor_interface_ros_tool/MotorCommand.h>
#include <motor_interface_ros_tool/ServoCommand.h>

namespace motor_interface_ros_tool {

class MotorInterfaceNode {
public:
    MotorInterfaceNode(ros::NodeHandle);

    inline ~MotorInterfaceNode() {
        motor.emergencyStop();
        servo.setSteerOff();
    }

    void onMotorCommand(const MotorCommand::ConstPtr&);
    void onServoCommand(const ServoCommand::ConstPtr&);
    void onMotorUpdate(const ros::TimerEvent& e);
    bool activateService(Activation::Request& request, Activation::Response& response);

private:
    unsigned char fromAngle(const double angle);
    inline double fromTicks(const unsigned char ticks) {
        return (static_cast<double>(MAX_TICKS - ticks) - steer_offset) * steer_ratio;
    }

    static const unsigned char MAX_TICKS = 255;
    static const unsigned char MIN_TICKS = 0;

    // parameters
    const double gear_ratio;   ///< gear ratio: 1 revolution covers 'gearRatio' m
    const double steer_ratio;  ///< steer ratio: 1 tick covers 'steerRatio' radians
    const double steer_offset; ///< 'steerOffset' ticks mean zero degree
    const ros::Rate maximal_update_rate;

    bool is_activated;

    double velocity;
    double steering;

    motor_interface::EPOS motor;
    motor_interface::Servo servo;

    ros::Subscriber rossub_motor_command;
    ros::Subscriber rossub_servo_command;
    ros::ServiceServer rosserv_activation;
    ros::Timer motor_update_timer;
};
}

#endif // MOTORINTERFACENODE_H
