#include "lateral_controller.hpp"
#include <tf2_eigen/tf2_eigen.h>
#include <utils_ros/ros_console.hpp>
#include <boost/algorithm/clamp.hpp>
#include <boost/math/special_functions/sign.hpp>
#include <boost/range/algorithm/min_element.hpp>

#include "lateral_control_ros_tool/discrete_curvature.h"
#include "motor_interface_ros_tool/ServoCommand.h"
#include "safe_iterator_operations.h"

namespace lateral_control_ros_tool {

using ServoCommand = motor_interface_ros_tool::ServoCommand;

LateralController::LateralController(ros::NodeHandle nh_public, ros::NodeHandle nh_private)
        : params_{nh_private}, reconfigureServer_{nh_private}, tfListener_{tfBuffer_} {

    /**
     * Initialization
     */
    utils_ros::setLoggerLevel(nh_private);
    params_.fromParamServer();

    /**
     * Publishers & subscriber
     */

    servo_command_publisher_ = nh_private.advertise<ServoCommand>(params_.servo_command_topic, params_.msg_queue_size);

    // Instantiate subscriber last, to assure all objects are initialised when first message is received.
    path_subscriber_ = nh_private.subscribe(params_.path_topic,
                                            params_.msg_queue_size,
                                            &LateralController::pathCallback,
                                            this,
                                            ros::TransportHints().tcpNoDelay());
    control_loop_timer_ =
        nh_private.createTimer(ros::Rate(params_.control_loop_rate), &LateralController::controlLoopCallback, this);

    /**
     * Set up dynamic reconfiguration
     */
    reconfigureServer_.setCallback(boost::bind(&LateralController::reconfigureRequest, this, _1, _2));

    utils_ros::showNodeInfo();
}

void LateralController::pathCallback(const nav_msgs::Path::ConstPtr& msg) {
    path_.clear();
    path_.reserve(msg->poses.size());
    for (const auto& pose_stamped : msg->poses) {
        Eigen::Affine3d pose;
        tf2::fromMsg(pose_stamped.pose, pose);
        path_.push_back(pose);
    }
}

double signedAngleBetween(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
    const double vz = boost::math::sign(a.cross(b).z());
    return vz * std::acos(a.normalized().dot(b.normalized()));
}

void LateralController::controlLoopCallback(const ros::TimerEvent& timer_event) {
    if (path_.size() < 5) {
        ROS_INFO_STREAM("No Path received yet");
        return;
    }

    /*
     * Lookup the latest transform from vehicle to map frame
     */
    Eigen::Affine3d vehicle_pose;
    try {
        const geometry_msgs::TransformStamped tf_ros =
            tfBuffer_.lookupTransform(params_.map_frame_id, params_.vehicle_frame_id, ros::Time(0));
        vehicle_pose = tf2::transformToEigen(tf_ros);
    } catch (const tf2::TransformException& e) {
        ROS_WARN_STREAM(e.what());
        return;
    }

    const Eigen::Vector3d vehicle_position = vehicle_pose.translation();

    /*
     * Shift Rear-axle in current direction -> kos_shift
     */
    const Eigen::Vector3d vehicle_frame_unit_x = [&vehicle_pose]() {
        Eigen::Vector3d p = vehicle_pose.rotation() * Eigen::Vector3d::UnitX();
        p.z() = 0.0;
        return p.normalized();
    }();

    const Eigen::Vector3d shifted_vehicle_position = vehicle_position + vehicle_frame_unit_x * params_.kos_shift;


    auto const& it = boost::range::min_element(
        path_, [&shifted_vehicle_position](const Eigen::Affine3d& lhs, const Eigen::Affine3d& rhs) {
            return (lhs.translation() - shifted_vehicle_position).squaredNorm() <
                   (rhs.translation() - shifted_vehicle_position).squaredNorm();
        });

    if (it == std::prev(path_.end())) {
      ROS_ERROR("Reached end of trajectory!");
      return;
    }

    const Eigen::Vector3d closest_trajectory_point = it->translation();
    /**
     * Find look ahead point
     */
    //	params_.index_shift --- lookaheadpoint
    //	params_.ii_off ---	point offset for curvature approximation
    auto const& it_lb = safe_next(it, params_.index_shift, std::prev(path_.end()));
    const Eigen::Vector3d& p_lookahead = it_lb->translation();

    // Determine three points for approximation
    const Eigen::Vector3d& p_prev = safe_prev(it_lb, params_.ii_off, path_.begin())->translation();
    const Eigen::Vector3d& p_next = safe_next(it_lb, params_.ii_off, std::prev(path_.end()))->translation();

    const double curv = discreteCurvature(p_prev.head<2>(), p_lookahead.head<2>(), p_next.head<2>());

    const Eigen::Vector3d target_direction = p_next - p_lookahead;

    /*
     * Compute angle difference
     */
    const double delta_angle = signedAngleBetween(target_direction, vehicle_frame_unit_x);
    /*
     * Calculation of the sign for distance control (= determine side of trajectory)
     */
    const double vz_dist =
        boost::math::sign(target_direction.cross(closest_trajectory_point - shifted_vehicle_position).z());
    const double dist = (closest_trajectory_point - shifted_vehicle_position).norm();

    /*
     * Controller law
     */
    double r_ang = -1. * params_.k_ang * delta_angle;
    double r_dist = params_.k_dist * vz_dist * dist;
    double steering_angle = std::atan(params_.wheel_base * (curv + r_ang + r_dist));

    steering_angle = boost::algorithm::clamp(steering_angle, -params_.max_steering_angle, params_.max_steering_angle);

    ROS_DEBUG_STREAM("Steering_angle: " << steering_angle);

    ServoCommand servo_command;
    servo_command.header.stamp = timer_event.current_expected;
    servo_command.steering_angle = steering_angle;

    servo_command_publisher_.publish(servo_command);
}

/**
  * This callback is called at startup or whenever a change was made in the dynamic_reconfigure window
*/
void LateralController::reconfigureRequest(const Config& config, uint32_t level) {
    params_.fromConfig(config);
}


} // namespace lateral_control_ros_tool
