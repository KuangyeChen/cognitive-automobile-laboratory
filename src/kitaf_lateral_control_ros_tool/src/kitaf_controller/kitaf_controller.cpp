#include "kitaf_controller.hpp"

#include <tf2_eigen/tf2_eigen.h>
#include <boost/algorithm/clamp.hpp>
#include <boost/math/special_functions/sign.hpp>
#include <boost/math/complex/atan.hpp>
#include <tf/tfMessage.h>
#include <geometry_msgs/Vector3.h>
#include <boost/range/algorithm/min_element.hpp>


#include "motor_interface_ros_tool/ServoCommand.h"
#include "motor_interface_ros_tool/MotorCommand.h"
#include "kitaf_safe_iterator_operations.h"
#include "kitaf_controller/kitaf_lateral_control_bag.hpp"

namespace kitaf_lateral_control_ros_tool {

using namespace kitaf_utils_ros;
using ServoCommand = motor_interface_ros_tool::ServoCommand;
using MotorCommand = motor_interface_ros_tool::MotorCommand;

KitafController::KitafController(ros::NodeHandle nhPublic, ros::NodeHandle nhPrivate)
        : interface_{nhPrivate}, reconfigureServer_{nhPrivate}, StopSignal{10000},
          near_center(true), pass_center(true), ready_watch(false), exit_state(false),
          lastSteeringAngle(0.0), lastYawAngle(0.0), k_velocity(1),
          currentTask(0), k_yaw(0), k_yaw_error(0), k_dis(0),
          velocity(0), tfListener_{tfBuffer_} {
    /**
     * Initialization
     */
    interface_.fromParamServer();

     /**
	 * Publishers & subscriber
	 */
     servo_command_publisher_ = nhPrivate.advertise<ServoCommand>(interface_.servo_topic,
                                                                  interface_.msg_queue_size);
     motor_command_publisher_ = nhPrivate.advertise<MotorCommand>(interface_.motor_topic,
                                                                  interface_.msg_queue_size);

     path_subscriber_ = nhPrivate.subscribe(interface_.path_topic,
                                            interface_.msg_queue_size,
                                            &KitafController::pathCallback,
                                            this,
                                            ros::TransportHints().tcpNoDelay());

     debug_publisher_ = nhPrivate.advertise<geometry_msgs::Vector3>("/control_debug",
                                                                    interface_.msg_queue_size);

     stopSign_subcriber_ = nhPrivate.subscribe(interface_.stop_sign_topic,
                                              interface_.msg_queue_size,
                                              &KitafController::StopSignCallback,
                                              this,
                                              ros::TransportHints().tcpNoDelay());
     motor_state_subscriber_ = nhPrivate.subscribe(interface_.motor_state_topic,
                                                   interface_.msg_queue_size,
                                                   &KitafController::motorStateCallback,
                                                   this,
                                                   ros::TransportHints().tcpNoDelay());
     currentTask_subscriber_ = nhPrivate.subscribe(interface_.currentTask_topic,
                                                   interface_.msg_queue_size,
                                                   &KitafController::currentTaskCallback,
                                                   this,
                                                   ros::TransportHints().tcpNoDelay());
     control_loop_timer_ = nhPrivate.createTimer(ros::Rate(interface_.control_loop_rate),
                                                 &KitafController::controlLoopCallback,
                                                 this);
     velocity_loop_timer_= nhPrivate.createTimer(ros::Rate(interface_.velocity_loop_rate),
                                                &KitafController::velocityLoopCallback,
                                                 this);

    /**
     * Set up callbacks for subscribers and reconfigure.
     */
    reconfigureServer_.setCallback(boost::bind(&KitafController::reconfigureRequest, this, _1, _2));
    rosinterface_handler::showNodeInfo();
}

void KitafController::currentTaskCallback(const std_msgs::Int8::ConstPtr& msg)
{
    currentTask = msg-> data;
}

void KitafController::motorStateCallback(const MotorState::ConstPtr& msg)
{
    motor_state_[0] = msg -> velocity;
    motor_state_[1] = msg -> steering_angle;
}

double signedAngleBetween(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
	const double vz = boost::math::sign(a.cross(b).z());
	return vz * std::acos(a.normalized().dot(b.normalized()));
}

void KitafController::pathCallback(const nav_msgs::Path::ConstPtr& msg)
{
    path_.clear();
    path_.reserve(msg->poses.size());
    for (const auto& pose_stamped : msg->poses) {
        Eigen::Affine3d pose;
        tf2::fromMsg(pose_stamped.pose, pose);
        path_.push_back(pose);
    }

}


void KitafController::velocityLoopCallback(const ros::TimerEvent &)
{
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

    Eigen::Vector3d center_(7.1, 3.0, 0);

    const double distance2center = (vehicle_position - center_).norm();

    if (!near_center) {
        near_center = distance2center < interface_.slowDistance;
        return;
    }
    if (!pass_center) {
        pass_center = distance2center > interface_.slowDistance;
        return;
    }
    if (!ready_watch) {
        ready_watch = distance2center < interface_.slowDistance;
        return;
    }

    k_velocity = 0.5;

    if (StopSignal < interface_.stopDistance)
    {
        ROS_INFO_STREAM("Stop!!!");
        k_velocity = 0;
        near_center = false;
        pass_center = false;
        ready_watch = false;
        return;
    }

    if (distance2center < 1.5)
    {
        ROS_INFO_STREAM("Small Circle!!");
        k_velocity = 1;
        near_center = false;
        pass_center = false;
        ready_watch = false;
    }

}

void KitafController::controlLoopCallback(const ros::TimerEvent& timer_event) {

    if (path_.size() < 5) {
        ROS_WARN_STREAM_THROTTLE(1.5, "No Path received.");
        return;
    }

    //longitudinal control, publish motor command
    MotorCommand motor_command;
    motor_command.header.stamp = ros::Time::now();
    //lateral control, publish servo command
    ServoCommand servo_command;
    servo_command.header.stamp = ros::Time::now();


    if (k_velocity == 0)
    {
        motor_command.velocity = 0.0;
        motor_command_publisher_.publish(motor_command);

        ros::Duration(2).sleep();
        k_velocity = 1;
        return;
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

    Eigen::Vector3d current_vehicle_position = vehicle_pose.translation();
    Eigen::Vector3d exit_position (13.111, 0.708, 0);
    double distance_exit = (current_vehicle_position - exit_position).norm();

    if (currentTask == 1)
    {
        if ( !exit_state ) {
            exit_state = distance_exit < 0.6;
        }
        ROS_DEBUG_STREAM("--------parameter for exit path---------");
        k_yaw = interface_.k_yaw_exit;
        k_yaw_error = interface_.k_yaw_error_exit;
        k_dis = interface_.k_dis_exit;
        velocity = exit_state ? interface_.velocity_exit : 0.5;
    }
    else
    {
        ROS_DEBUG_STREAM("-------default parameter-------");
        k_yaw = interface_.k_yaw;
        k_yaw_error = interface_.k_yaw_error;
        k_dis = interface_.k_dis;
        velocity = interface_.velocity_cur;
    }

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
    auto const &it_current = boost::range::min_element(
            path_, [&current_vehicle_position](const Eigen::Affine3d &lhs, const Eigen::Affine3d &rhs) {
                return (lhs.translation() - current_vehicle_position).squaredNorm() <
                       (rhs.translation() - current_vehicle_position).squaredNorm();
            });

    //calculate yaw angle of current point in th path
    auto it1 = it_current;
    it1 = it_current == path_.begin() ? it_current + 1 : it_current;
    it1 = it_current == path_.end() ? it_current - 1 : it_current;
    Eigen::Vector3d it1_1 = it1->translation();
    Eigen::Vector3d it1_2 = (it1 + 1)->translation();

    Eigen::Vector3d cur_trajectory_direction = it1_2 - it1_1;
    Eigen::Vector3d closest_trajectory_point = it_current->translation();
    closest_trajectory_point.z() = 0;

    /**
     * Shift vehicle into future.
     */
    Eigen::Vector3d shift_vehicle_position = current_vehicle_position + interface_.k_shift * vehicle_frame_unit_x;

    /**
     * find the closest shifted point in the path
     */
    auto const &it_vor = boost::range::min_element(
            path_, [&shift_vehicle_position](const Eigen::Affine3d &lhs, const Eigen::Affine3d &rhs) {
                return (lhs.translation() - shift_vehicle_position).squaredNorm() <
                       (rhs.translation() - shift_vehicle_position).squaredNorm();
            });

    //calculate yaw angle of shift point in th path
    auto it = it_vor;
    //check, whether 'it_vor' is the first point or the last point of the path
    it = it_vor == path_.begin() ? it_vor + 1 : it_vor; //if 'it_vor' is the first point of the path: it= it_vor+1
    it = it_vor == path_.end() ? it_vor - 1 : it_vor;
    Eigen::Vector3d it_1 = it->translation();
    Eigen::Vector3d it_2 = (it + 1)->translation();

    Eigen::Vector3d shift_closest_trajectory_point = it_vor->translation();
    shift_closest_trajectory_point.z() = 0;

    Eigen::Vector3d shift_trajectory_direction = it_2 - it_1;

   /**
	* Stanley Controller Law with Dynamic model
	*/
    //calculate the error direction between vehicle position to error position
    Eigen::Vector3d error_direction = closest_trajectory_point - current_vehicle_position;
    error_direction.z()=0;
    double error_distance = error_direction.norm();
    //sign of the cross product between car direction and error direction
    const double vz = boost::math::sign(vehicle_frame_unit_x.cross(error_direction).z());
    //calculate the angle between error direction and car direction
    //calculate the yaw angle
//    double yaw_angle = signedAngleBetween(vehicle_frame_unit_x , shift_trajectory_direction);
    double yaw_angle = signedAngleBetween(vehicle_frame_unit_x , cur_trajectory_direction);

    //calculate the trajectory yaw rate
    double yaw_error = signedAngleBetween(cur_trajectory_direction , shift_trajectory_direction);
    double yaw_angle_error = fabs(yaw_error * 180 / M_PI) > interface_.error_interval ? yaw_error : 0;

   /**
 	* calculate the steering angle of front axle
 	*/

   double steering_angle_kinmatic =  k_yaw * yaw_angle +
                          atan(k_dis * vz * error_distance /(k_velocity * velocity/cos(lastSteeringAngle)));

   double steering_angle = steering_angle_kinmatic +
                            k_yaw_error * yaw_angle_error - interface_.k_compensation;

    // steering angle should be in interval [-max,max] max is 0.7 in radian
    steering_angle_kinmatic = boost::algorithm::clamp(steering_angle_kinmatic, -interface_.max_steering_angle, interface_.max_steering_angle);
    steering_angle = boost::algorithm::clamp(steering_angle, -interface_.max_steering_angle, interface_.max_steering_angle);


    ROS_DEBUG_STREAM("parameter value------------------------"<<std::endl
                     << "---------yaw_angle item : "<<  interface_.k_yaw * yaw_angle* 180 / M_PI<<std::endl
                     << "---------error item : "<< atan(k_dis * vz * error_distance /(velocity/cos(lastSteeringAngle)))* 180 / M_PI<<std::endl
                     << "---------rate item : "<<   k_yaw_error * yaw_angle_error * 180 / M_PI <<std::endl
                     << "---------yaw_angle : "<< yaw_angle* 180 / M_PI<<std::endl
                     << "---------error distance : "<< vz*error_distance <<std::endl
                     << "---------yaw_error : "<< yaw_error * 180 / M_PI <<std::endl
                     << "---------steering_kinematic : "<<steering_angle_kinmatic* 180 / M_PI<<std::endl
                     << "---------steering_dynamic : "<<steering_angle* 180 / M_PI<<std::endl
    );

    motor_command.velocity = k_velocity * velocity;
    motor_command_publisher_.publish(motor_command);

    servo_command.steering_angle = steering_angle;
    servo_command_publisher_.publish(servo_command);

    geometry_msgs::Vector3 debug_msg;
    debug_msg.x = steering_angle_kinmatic* 180 / M_PI;
    debug_msg.y = steering_angle* 180 / M_PI;
    debug_msg.z = signedAngleBetween(current_vehicle_position, Eigen::Vector3d::UnitX()) * 180 / M_PI;
    debug_publisher_.publish(debug_msg);

    lastSteeringAngle = steering_angle;
    lastYawAngle = yaw_angle;
}

void KitafController::StopSignCallback(const std_msgs::Float64::ConstPtr& msg)
{
    StopSignal = msg->data;
}

/**
  * This callback is called at startup or whenever a change was made in the dynamic_reconfigure window
*/
void KitafController::reconfigureRequest(const ReconfigureConfig& config, uint32_t level) {
    interface_.fromConfig(config);
}


} // namespace kitaf_lateral_control_ros_tool
