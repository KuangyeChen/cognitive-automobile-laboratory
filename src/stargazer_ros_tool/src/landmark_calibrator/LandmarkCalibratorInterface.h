//
// Created by bandera on 10.06.16.
//

#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <rosbag/bag.h>
#include <stargazer_ros_tool/Landmark.h>
#include "LandmarkCalibratorInterfaceParameters.h"
#include "stargazer/LandmarkCalibrator.h"
#include "stargazer/StargazerImgTypes.h"
#include "stargazer_ros_tool/LandmarkArray.h"

namespace stargazer_ros_tool {

class LandmarkCalibratorInterface {
public:
    LandmarkCalibratorInterface(ros::NodeHandle, ros::NodeHandle);
    ~LandmarkCalibratorInterface();

private:
    LandmarkCalibratorInterfaceParameters& params_;
    std::unique_ptr<stargazer::LandmarkCalibrator> bundleAdjuster;
    std::vector<stargazer::pose_t> observed_poses;
    std::vector<std::vector<stargazer::ImgLandmark>> observed_landmarks;
    std::vector<ros::Time> observed_timestamps;
    std::string pose_frame;
    rosbag::Bag bag_out;

    void load_data();
    void write_data();
    void optimize();
    void synchronizerCallback(const stargazer_ros_tool::LandmarkArray::ConstPtr& lm_msg,
                              const geometry_msgs::PoseStamped::ConstPtr& pose_msg);
};

} // namespace stargazer_ros_tool
