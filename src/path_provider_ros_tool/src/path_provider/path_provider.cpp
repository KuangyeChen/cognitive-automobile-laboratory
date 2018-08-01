#include "path_provider.hpp"

#include <eigen_conversions/eigen_msg.h>
#include <tf/tfMessage.h>
#include <utils_ros/ros_console.hpp>

#include "bag.hpp"

namespace path_provider_ros_tool {

PathProvider::PathProvider(ros::NodeHandle nh_public, ros::NodeHandle nh_private)
        : params_{nh_private}, reconfigure_server_{nh_private}, tf_listener_{tf_buffer_} {

    /**
     * Initialization
     */
    utils_ros::setLoggerLevel(nh_private);
    params_.fromParamServer();

    /**
     * Set up dynamic reconfiguration
     */
    reconfigure_server_.setCallback(boost::bind(&Parameters::fromConfig, &params_, _1, _2));

    /**
     * Read the rosbag with tf messages
     */
    const std::vector<tf::tfMessage::ConstPtr> msgs_tfs = fromBag<tf::tfMessage>(params_.bag_file_name, "/tf");

    /**
     * Try to find a transform between stargazer and map
     * Here, wait for three seconds and cancel if no transform was found by then.
     */
    Eigen::Affine3d tf_stargazer_to_map_eigen, tf_vehicle_to_camera_eigen;
    try {
        ros::Time now = ros::Time::now();
        const geometry_msgs::TransformStamped tf_stargazer_to_map_ros{tf_buffer_.lookupTransform(
            params_.frame_id_map, params_.frame_id_stargazer, ros::Time(0), ros::Duration(3))};
        tf::transformMsgToEigen(tf_stargazer_to_map_ros.transform, tf_stargazer_to_map_eigen);
        const geometry_msgs::TransformStamped tf_vehicle_to_camera_ros{tf_buffer_.lookupTransform(
            params_.frame_id_camera, params_.frame_id_vehicle, ros::Time(0), ros::Duration(3))};
        tf::transformMsgToEigen(tf_vehicle_to_camera_ros.transform, tf_vehicle_to_camera_eigen);
    } catch (const tf2::TransformException& ex) {
        ROS_ERROR_STREAM(ex.what());
        ros::shutdown();
    }

    path_->header.frame_id = params_.frame_id_map;
    path_->header.stamp = ros::Time::now();
    geometry_msgs::PoseStamped pose_ros;
    pose_ros.header = path_->header;
    size_t count{0};
    for (auto const& msg_tfs : msgs_tfs) {
        Eigen::Affine3d tf_camera_to_stargazer_eigen;
        for (auto const& tf : msg_tfs->transforms) {
            if (tf.header.frame_id == params_.frame_id_stargazer && tf.child_frame_id == params_.frame_id_camera)
                tf::transformMsgToEigen(tf.transform, tf_camera_to_stargazer_eigen);
            else if (tf.header.frame_id == params_.frame_id_camera && tf.child_frame_id == params_.frame_id_stargazer) {
                tf::transformMsgToEigen(tf.transform, tf_camera_to_stargazer_eigen);
                tf_camera_to_stargazer_eigen = tf_camera_to_stargazer_eigen.inverse();
            } else
                continue;
            const Eigen::Affine3d tf_vehicle_to_map_eigen{tf_stargazer_to_map_eigen * tf_camera_to_stargazer_eigen *
                                                          tf_vehicle_to_camera_eigen};
            poses_.emplace(tf.header.stamp.toSec(), tf_vehicle_to_map_eigen);
            tf::poseEigenToMsg(tf_vehicle_to_map_eigen, pose_ros.pose);
            path_->poses.emplace_back(pose_ros);
            count++;
        }
    }
    ROS_DEBUG_STREAM("Found " << count << " poses in rosbag '" << params_.bag_file_name << "'.");
    publisher_path_ = nh_private.advertise<nav_msgs::Path>(params_.topic_publisher, params_.msg_queue_size);

    /**
     * Set up the timer running at a fixed rate, calling "callbackTimer"
     */
    timer_ = nh_private.createTimer(ros::Rate(params_.timer_rate), &PathProvider::callbackTimer, this);

    /**
     * Show info after everything is started
     */
    utils_ros::showNodeInfo();
}

void PathProvider::callbackTimer(const ros::TimerEvent& timer_event) {
    path_->header.stamp = timer_event.current_expected;
    publisher_path_.publish(path_);
}
}
