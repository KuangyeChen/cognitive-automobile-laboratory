#ifndef TEXTBASEDUSERINTERFACENODE_H
#define TEXTBASEDUSERINTERFACENODE_H

#include <termios.h>
#include <motor_interface_ros_tool/MotorCommand.h>
#include <motor_interface_ros_tool/ServoCommand.h>
#include <ros/ros.h>

namespace tui_ros_tool {

class TUINode {
public:
    TUINode(ros::NodeHandle);
    inline ~TUINode() {
        tcsetattr(STDIN_FILENO, TCSANOW, &save_termattr);
    }

    void runOnce();

    int spin();

private:
    static struct termios getTermAttr();
    static struct termios configureTermAttr(const termios&);

    void infoText() const;

    void activateMotorInterface(const bool activate);
    void changeVelocity(const double diff);
    void changeSteering(const double diff);

    ros::Rate loop_rate;
    double max_steering_angle;

    struct termios save_termattr;
    struct termios termattr;

    motor_interface_ros_tool::MotorCommand last_motor_command;
    motor_interface_ros_tool::ServoCommand last_servo_command;

    ros::ServiceClient rosclient_activation;
    ros::Publisher rospub_motor_command;
    ros::Publisher rospub_servo_command;
};

} // namespace tui_ros_tool

#endif // TEXTBASEDUSERINTERFACENODE_H
