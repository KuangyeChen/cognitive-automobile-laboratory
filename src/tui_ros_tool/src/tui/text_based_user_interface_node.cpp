#include "text_based_user_interface_node.hpp"

#include <fcntl.h>
#include <motor_interface_ros_tool/Activation.h>
#include <ros/ros.h>
#include <utils_ros/node_handle.hpp>
#include <boost/algorithm/clamp.hpp>

namespace tui_ros_tool {

TUINode::TUINode(ros::NodeHandle nh)
        : loop_rate(utils_ros::getParam<double>(nh, "loop_rate")),
          max_steering_angle(utils_ros::getParam<double>(nh, "max_steering_angle")),
          save_termattr(getTermAttr()),
          termattr(configureTermAttr(save_termattr)) {
    tcsetattr(STDIN_FILENO, TCSANOW, &termattr);
    fcntl(STDIN_FILENO, F_SETFL, fcntl(STDIN_FILENO, F_GETFL) | O_NONBLOCK);

    rosclient_activation = nh.serviceClient<motor_interface_ros_tool::Activation>("activate");
    rospub_motor_command = nh.advertise<motor_interface_ros_tool::MotorCommand>("motor_command", 5);
    rospub_servo_command = nh.advertise<motor_interface_ros_tool::ServoCommand>("servo_command", 5);
}

void TUINode::runOnce() {
    unsigned char c = '.';
    ssize_t size = read(STDIN_FILENO, &c, 1);
    if (size > 0) {
        switch (c) {
        case 'Q':
        case 'q': // Programm beenden
            ROS_INFO("Quit.");
            ros::shutdown();
            break;
        case 'g': // Fahrzeug aktivieren
            activateMotorInterface(true);
            break;
        case ' ': // Fahrzeug deaktivieren
            activateMotorInterface(false);
            break;
        case '+': // Fahrzeug beschleunigen
            changeVelocity(0.1);
            break;
        case '-': // Fahrzeug abbremsen/rueckwaertsfahren
            changeVelocity(-0.1);
            break;
        case 'y': // nach links lenken
            changeSteering(M_PI / 180 * 5);
            break;
        case 'x': // nach rechts lenken
            changeSteering(M_PI / 180 * -5);
            break;
        case 'H':
        case 'h': // help
            infoText();
            break;
        default:
            break;
        }
    }
}

int TUINode::spin() {
    infoText();
    while (ros::ok()) {
        ros::spinOnce();
        runOnce();
        loop_rate.sleep();
    }
    return 0;
}

termios TUINode::getTermAttr() {
    termios save_termattr;
    tcgetattr(STDIN_FILENO, &save_termattr);
    return save_termattr;
}

termios TUINode::configureTermAttr(const termios& old_termattr) {
    termios new_termattr = old_termattr;
    new_termattr.c_lflag &= ~ICANON;
    new_termattr.c_lflag &= ~ECHO;
    return new_termattr;
}

void TUINode::infoText() const {
    // clang-format off
    ROS_INFO_STREAM("\n[Tastatur-Codes]\n"
    		<< "  'q' beendet das Programm \n"
			<< "  'h' liefert diesen Hilfetext \n"
			<< "  'g' Fahrzeug aktivieren \n"
			<< "  <blank> Fahrzeug deaktivieren \n"
			<< "  '+' Fahrzeug beschleunigen \n"
			<< "  '-' Fahrzeug abbremsen \n"
			<< "  'y' nach links lenken \n"
			<< "  'x' nach rechts lenken \n"
	);
    // clang-format on
}

void TUINode::activateMotorInterface(const bool activate) {
    if (activate)
        ROS_INFO("[Activate]");
    else
        ROS_INFO("[Deactivate]");

    motor_interface_ros_tool::Activation activation_call;
    activation_call.request.activate = activate;
    rosclient_activation.call(activation_call);
}

void TUINode::changeVelocity(const double diff) {
    last_motor_command.velocity += diff;
    last_motor_command.header.stamp = ros::Time::now();
    rospub_motor_command.publish(last_motor_command);
    ROS_INFO_STREAM("[Velocity=" << last_motor_command.velocity << " m/s]");
}

void TUINode::changeSteering(const double diff) {
    double new_angle = std::fmod(last_servo_command.steering_angle + diff, 2 * M_PI);
    new_angle = (new_angle < M_PI ? (new_angle > -M_PI ? new_angle : new_angle + 2 * M_PI)
                                  : new_angle - 2 * M_PI);
    last_servo_command.steering_angle = boost::algorithm::clamp(new_angle, -max_steering_angle, max_steering_angle);
    last_servo_command.header.stamp = ros::Time::now();
    rospub_servo_command.publish(last_servo_command);
    ROS_INFO_STREAM("[Steering Angle=" << last_servo_command.steering_angle << " rad]");
}

} // namespace tui_ros_tool

int main(int argc, char** argv) {
    ros::init(argc, argv, "motor_interface");
    tui_ros_tool::TUINode node(ros::NodeHandle("~"));
    return node.spin();
}
