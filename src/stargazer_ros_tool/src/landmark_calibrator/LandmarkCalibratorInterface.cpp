//
// Created by bandera on 10.06.16.
//

#include "LandmarkCalibratorInterface.h"

// ROS includes
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

// ROS messages
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

// Local Helpers
#include <boost/foreach.hpp>
#include <tf/transform_datatypes.h>
#include "../StargazerConversionMethods.h"
#include "../ros_utils.h"
#include "stargazer/StargazerConfig.h"
#define foreach BOOST_FOREACH

using namespace stargazer_ros_tool;

/**
 * Inherits from message_filters::SimpleFilter<M>
 * to use protected signalMessage function
 */
template <class M>
class BagSubscriber : public message_filters::SimpleFilter<M> {
public:
    void newMessage(const boost::shared_ptr<M const>& msg) {
        this->signalMessage(msg);
    }
};

LandmarkCalibratorInterface::LandmarkCalibratorInterface(ros::NodeHandle node_handle,
                                                         ros::NodeHandle private_node_handle)
        : params_{LandmarkCalibratorInterfaceParameters::getInstance()} {

    showNodeInfo();

    // Set parameters
    params_.fromNodeHandle(private_node_handle);
    bag_out.open(params_.bag_file + "_optimized.bag", rosbag::bagmode::Write);
    bundleAdjuster = std::make_unique<stargazer::LandmarkCalibrator>(params_.stargazer_cfg_file_in);
    load_data();

    // Init logging for ceres
    google::InitGoogleLogging("LandmarkCalibrator");
    FLAGS_logtostderr = 1;

    // Optimize
    optimize();

    // Write data
    write_data();
}

LandmarkCalibratorInterface::~LandmarkCalibratorInterface() {
    bag_out.close();
}

void LandmarkCalibratorInterface::synchronizerCallback(const stargazer_ros_tool::LandmarkArray::ConstPtr& lm_msg,
                                                       const geometry_msgs::PoseStamped::ConstPtr& pose_msg) {

    std::vector<stargazer::ImgLandmark> img_lms = convert2ImgLandmarks(*lm_msg);

    // Test for wrong number of detected LEDs
    size_t old_size = img_lms.size();
    img_lms.erase(std::remove_if(img_lms.begin(), img_lms.end(),
                                 [&](stargazer::ImgLandmark& lm) {
                                     return (lm.voCorners.size() + lm.voIDPoints.size() !=
                                             bundleAdjuster->getLandmarks().at(lm.nID).points.size());
                                 }),
                  img_lms.end());
    size_t new_size = img_lms.size();
    if (old_size != new_size)
        ROS_INFO_STREAM("Removed " << old_size - new_size << " landmarks because of wrong number of points.");
    if (!img_lms.empty()) {
        observed_timestamps.push_back(lm_msg->header.stamp);
        observed_landmarks.push_back(img_lms);
        observed_poses.push_back(gmPose2pose(pose_msg->pose));
        pose_frame = pose_msg->header.frame_id;

        bag_out.write(params_.pose_topic, pose_msg->header.stamp, pose_msg);
        bag_out.write(params_.landmark_topic, lm_msg->header.stamp, lm_msg);

    } else {
        ROS_WARN_STREAM("Received empty landmarks message.");
    }
}

void LandmarkCalibratorInterface::load_data() {
    ROS_INFO_STREAM("Reading bag file...");
    rosbag::Bag bag;
    bag.open(params_.bag_file, rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(std::string(params_.landmark_topic));
    topics.push_back(std::string(params_.pose_topic));

    // Set up fake subscribers to capture images
    BagSubscriber<stargazer_ros_tool::LandmarkArray> lm_sub;
    BagSubscriber<geometry_msgs::PoseStamped> pose_sub;

    // Use time synchronizer to make sure we get properly synchronized images
    message_filters::TimeSynchronizer<stargazer_ros_tool::LandmarkArray, geometry_msgs::PoseStamped> sync(lm_sub,
                                                                                                          pose_sub, 25);
    sync.registerCallback(boost::bind(&LandmarkCalibratorInterface::synchronizerCallback, this, _1, _2));

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    foreach (rosbag::MessageInstance const m, view) {

        if (m.isType<stargazer_ros_tool::LandmarkArray>()) {
            stargazer_ros_tool::LandmarkArray::ConstPtr lm_msg = m.instantiate<stargazer_ros_tool::LandmarkArray>();
            lm_sub.newMessage(lm_msg);
        } else if (m.isType<geometry_msgs::PoseStamped>()) {
            geometry_msgs::PoseStamped::ConstPtr pose_msg = m.instantiate<geometry_msgs::PoseStamped>();
            pose_sub.newMessage(pose_msg);
        }
    }

    bag.close();

    std::cout << "CameraParameters: " << bundleAdjuster->getIntrinsics().size() << std::endl;
    std::cout << "Landmarks: " << bundleAdjuster->getLandmarks().size() << std::endl;
    std::cout << "Observations(Images): " << observed_landmarks.size() << std::endl;
    std::cout << "Observations(Poses): " << observed_poses.size() << std::endl;
}

void LandmarkCalibratorInterface::write_data() {
    writeConfig(params_.stargazer_cfg_file_out, bundleAdjuster->getIntrinsics(), bundleAdjuster->getLandmarks());

    ROS_INFO_STREAM("Writing bag file..." << bag_out.getFileName());
    for (size_t i = 0; i < observed_timestamps.size(); i++) {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = observed_timestamps[i];
        pose.header.frame_id = pose_frame;
        pose.pose = pose2gmPose(bundleAdjuster->getPoses()[i]);
        bag_out.write(params_.pose_topic + "_optimized", observed_timestamps[i], pose);
    }
}

void LandmarkCalibratorInterface::optimize() {
    // Start work by setting up problem
    bundleAdjuster->AddReprojectionResidualBlocks(observed_poses, observed_landmarks);
    //    bundleAdjuster->SetPoseConstant(0); // First pose
    bundleAdjuster->SetLandmarkConstant(400); // First landmark in the lower left corner
                                              //    bundleAdjuster->SetParametersConstant();
    bundleAdjuster->Optimize();
}
