//
// Created by bandera on 09.06.16.
//

#include "LandmarkLocalizerInterface.h"
#include "../StargazerConversionMethods.h"
#include "../ros_utils.h"
#include "stargazer/CeresLocalizer.h"

using namespace stargazer_ros_tool;

LandmarkLocalizerInterface::LandmarkLocalizerInterface(ros::NodeHandle node_handle,
                                                       ros::NodeHandle private_node_handle)
        : server(private_node_handle) {

    // Set parameters
    params_.fromNodeHandle(private_node_handle);
    last_timestamp_ = ros::Time::now();
    debugVisualizer_.SetWaitTime(1);

    // Setup and set values in dynamic reconfigure server
    server.setCallback(boost::bind(&LandmarkLocalizerInterface::reconfigureCallback, this, _1, _2));

    localizer_ = std::make_unique<stargazer::CeresLocalizer>(params_.stargazer_config,
                                                             params_.estimate_2d_pose);

    // Initialize publisher
    pose_pub = node_handle.advertise<geometry_msgs::PoseStamped>(params_.pose_topic, 1);
    lm_sub = private_node_handle.subscribe<stargazer_ros_tool::LandmarkArray>(
        params_.landmark_topic, 1, &LandmarkLocalizerInterface::landmarkCallback, this);

    if (params_.debug_mode)
        showNodeInfo();
}

void LandmarkLocalizerInterface::landmarkCallback(
    const stargazer_ros_tool::LandmarkArray::ConstPtr& msg) {

    ros::Time this_timestamp = msg->header.stamp;
    double dt = (this_timestamp - last_timestamp_).toSec();
    ros::Time last_timestamp = this_timestamp;

    std::vector<stargazer::ImgLandmark> detected_landmarks = convert2ImgLandmarks(*msg);

    // Localize
    localizer_->UpdatePose(detected_landmarks, dt);
    stargazer::pose_t pose = localizer_->getPose();

    // Publish tf pose
    geometry_msgs::TransformStamped map2camTransform;
    pose2tf(pose, map2camTransform);
    map2camTransform.header.stamp = msg->header.stamp;
    map2camTransform.header.frame_id = params_.map_frame;
    map2camTransform.child_frame_id = params_.camera_frame;
    tf_pub.sendTransform(map2camTransform);

    geometry_msgs::PoseStamped poseStamped;
    poseStamped.header.frame_id = params_.map_frame;
    poseStamped.header.stamp = msg->header.stamp;
    poseStamped.pose = pose2gmPose(pose);
    pose_pub.publish(poseStamped);

    //  Visualize
    if (params_.debug_mode) {
        cv::Mat img = cv::Mat::zeros(1024, 1360, CV_8UC3);
        img.setTo(cv::Scalar(255, 255, 255));
        debugVisualizer_.DrawLandmarks(img, detected_landmarks);
        debugVisualizer_.DrawLandmarks(
            img, localizer_->getLandmarks(), localizer_->getIntrinsics(), pose);
        debugVisualizer_.ShowImage(img, "ReprojectionImage");

        // clang-format off
//        static int count = 0;
//        cv::imwrite((boost::format("/home/bandera/Documents/MRT/Papers/ITSC2016/StargazerPaper/pics/reprojection_%010d.jpg") % count).str(),img);
//        ++count;
        // clang-format on
    }

    const ceres::Solver::Summary& summary =
        dynamic_cast<stargazer::CeresLocalizer*>(localizer_.get())->getSummary();
    ROS_DEBUG_STREAM("Number of iterations: " << summary.iterations.size() << " Time needed: "
                                              << summary.total_time_in_seconds);
    if (summary.termination_type != ceres::TerminationType::CONVERGENCE) {
        ROS_WARN_STREAM("Solver did not converge! "
                        << ceres::TerminationTypeToString(summary.termination_type));
        ROS_WARN_STREAM(summary.FullReport());
    }
}

void LandmarkLocalizerInterface::reconfigureCallback(LandmarkLocalizerConfig& config,
                                                     uint32_t level) {
    params_.debug_mode = config.debug_mode;
}
