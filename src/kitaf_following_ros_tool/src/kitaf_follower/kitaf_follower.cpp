#include "kitaf_follower.hpp"

#include "motor_interface_ros_tool/ServoCommand.h"
#include "motor_interface_ros_tool/MotorCommand.h"
#include <tf2_eigen/tf2_eigen.h>
#include <tf/tfMessage.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>
#include <boost/algorithm/clamp.hpp>
#include <boost/range/algorithm/min_element.hpp>

namespace kitaf_following_ros_tool {

    using ServoCommand = motor_interface_ros_tool::ServoCommand;
    using MotorCommand = motor_interface_ros_tool::MotorCommand;

KitafFollower::KitafFollower(ros::NodeHandle nhPublic, ros::NodeHandle nhPrivate)
        : interface_{nhPrivate}, reconfigureServer_{nhPrivate}, tfListener_{tfBuffer_} {

    /**
     * Initialization
     */
    interface_.fromParamServer();
    posecall = false;

    /**
     * Publisher and subscriber
     */
    servo_command_publisher_ = nhPrivate.advertise<ServoCommand>(interface_.servo_topic,
                                                                 interface_.msg_queue_size);
    motor_command_publisher_ = nhPrivate.advertise<MotorCommand>(interface_.motor_topic,
                                                                 interface_.msg_queue_size);

    targetPose_subscriber_ = nhPrivate.subscribe(interface_.targetPose_topic,
                                           interface_.msg_queue_size,
                                           &KitafFollower::targetPoseCallback,
                                           this,
                                           ros::TransportHints().tcpNoDelay());

    control_loop_timer_ = nhPrivate.createTimer(ros::Rate(interface_.control_loop_rate),
                                                &KitafFollower::controlLoopCallback,
                                                this);

    /**
     * Set up callbacks for subscribers and reconfigure.
     *
     * New subscribers can be created with "add_subscriber" in "cfg/KitafFollower.if file.
     * Don't forget to register your callbacks here!
     */
    reconfigureServer_.setCallback(boost::bind(&KitafFollower::reconfigureRequest, this, _1, _2));
    interface_.dummy_subscriber->registerCallback(&KitafFollower::callbackSubscriber, this);

    rosinterface_handler::showNodeInfo();
}

void KitafFollower::targetPoseCallback(const geometry_msgs::PoseStamped::ConstPtr & msg){

    posecall = true;
    pose_(0) = msg->pose.position.x ;
    pose_(1) = msg->pose.position.y ;
    pose_(2) = 0;
}

double signedAngleBetween(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
    const double vz = boost::math::sign(a.cross(b).z());
    return vz * std::acos(a.normalized().dot(b.normalized()));
}

void KitafFollower::controlLoopCallback(const ros::TimerEvent& timer_event){

    if (!posecall){
        ROS_WARN_STREAM_THROTTLE(1.5, "No Pose received.");
        return;
    } else {
        posecall = false;
        ROS_DEBUG_STREAM("Pose Received: " << pose_);
    }

    /**
     * Lookup the latest transform from vehicle to map frame
     */
    Eigen::Affine3d vehicle_pose;
    try {
        //here needs the front axle, frame_id_vehicle = vehicle_front_axle, frame_id_map = world
        const geometry_msgs::TransformStamped tf_ros =
                tfBuffer_.lookupTransform(interface_.frame_id_map, interface_.frame_id_vehicle, ros::Time(0));
        vehicle_pose = tf2::transformToEigen(tf_ros);
    } catch (const tf2::TransformException &e) {
        ROS_WARN_STREAM(e.what());
        return;
    }
    Eigen::Vector3d vehicle_position = vehicle_pose.translation();

    /**
     * Current car direction in world frame
     */
    const Eigen::Vector3d vehicle_frame_unit_x = [&vehicle_pose]() {
        Eigen::Vector3d p = vehicle_pose.rotation() * Eigen::Vector3d::UnitX();
        p.z() = 0.0;
        return p.normalized();
    }();

    /**
     * find the closest point in the path
     */
//    auto const &it = boost::range::min_element(
//            path_, [&vehicle_position](const Eigen::Affine3d &lhs, const Eigen::Affine3d &rhs) {
//                return (lhs.translation() - vehicle_position).squaredNorm() <
//                       (rhs.translation() - vehicle_position).squaredNorm();
//            });
//    if (it == path_.end()){
//        ROS_WARN_STREAM("Reach the end of the path!!");
//        return;
//    }

    Eigen::Vector3d path_direction = pose_ - vehicle_position;
//    Eigen::Vector3d closest_trajectory_point = it->translation();

//    //calculate the error direction between vehicle position to error position
//    Eigen::Vector3d error_direction = closest_trajectory_point - vehicle_position;
//    error_direction.z()=0;
//    double error_distance = error_direction.norm();
//    //sign of the cross product between car direction and error direction
//    const double vz = boost::math::sign(vehicle_frame_unit_x.cross(error_direction).z());
    //calculate the auto yaw rate
    double yaw_angle = signedAngleBetween(vehicle_frame_unit_x , path_direction);

    /**
     * calculate the steering angle of front axle
     */
//    double steering_angle =  interface_.k_yaw * yaw_angle +
//                      atan(interface_.k_dis * vz * error_distance /interface_.velocity);

    double steering_angle =  interface_.k_yaw * yaw_angle;

    // steering angle should be in interval [-max,max] max is 0.7 in radian
    steering_angle = boost::algorithm::clamp(steering_angle, -interface_.max_steering_angle, interface_.max_steering_angle);


    MotorCommand motor_command;
    motor_command.header.stamp = ros::Time::now();
    ServoCommand servo_command;
    servo_command.header.stamp = ros::Time::now();

    if (path_direction.norm() < 1){
        ROS_DEBUG_STREAM("path distance is smaller than 1");
        motor_command.velocity = 0;
        servo_command.steering_angle = 0;
    }
    else{
        motor_command.velocity = interface_.velocity;
        servo_command.steering_angle = steering_angle;
    }

    ROS_DEBUG_STREAM(std::endl<<"----------PARAMETER-----------"<<std::endl
                              <<"velocity:"<< motor_command.velocity<<std::endl
                              <<"steering angle:"<<servo_command.steering_angle<<std::endl
                              <<"------------------------------"<<std::endl);
    motor_command_publisher_.publish(motor_command);
    servo_command_publisher_.publish(servo_command);
}


void KitafFollower::callbackSubscriber(const Msg::ConstPtr& msg) {

    // do your stuff here...
    Msg::Ptr newMsg = boost::make_shared<Msg>(*msg);
    interface_.dummy_publisher.publish(newMsg);
}

/**
  * This callback is called at startup or whenever a change was made in the dynamic_reconfigure window
*/
void KitafFollower::reconfigureRequest(const ReconfigureConfig& config, uint32_t level) {
    interface_.fromConfig(config);
}


} // namespace kitaf_following_ros_tool
