//
// Created by bandera on 20.03.16.
//
#include <boost/foreach.hpp>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include "PoseVisualizerParameters.h"
#include "../StargazerConversionMethods.h"
#include "../ros_utils.h"
#include "stargazer/CeresLocalizer.h"
#include "stargazer_ros_tool/LandmarkArray.h"
#define foreach BOOST_FOREACH

using namespace stargazer;
using namespace stargazer_ros_tool;

int main(int argc, char** argv) {
    ros::init(argc, argv, "pose_visualizer");
    ros::NodeHandle n, np("~");

    PoseVisualizerParameters& params = PoseVisualizerParameters::getInstance();
    params.fromNodeHandle(np);

    ros::Publisher path_pub = n.advertise<geometry_msgs::PoseArray>(params.pose_pub_topic, 1);
    std::unique_ptr<stargazer::CeresLocalizer> localizer =
        std::make_unique<stargazer::CeresLocalizer>(params.stargazer_config);

    geometry_msgs::PoseArray pose_array;

    rosbag::Bag bag(params.bag_file, rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(std::string(params.landmark_topic));
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    foreach (rosbag::MessageInstance const m, view) {
        stargazer_ros_tool::LandmarkArray::ConstPtr img_landmark_msg =
            m.instantiate<stargazer_ros_tool::LandmarkArray>();
        std::vector<stargazer::ImgLandmark> img_landmarks = convert2ImgLandmarks(*img_landmark_msg);
        localizer->UpdatePose(img_landmarks, 0.0);
        geometry_msgs::PoseStamped camera_pose;
        camera_pose.pose = pose2gmPose(localizer->getPose());
        pose_array.poses.push_back(camera_pose.pose);
    }
    bag.close();
    pose_array.header.frame_id = params.map_frame;

    showNodeInfo();

    ros::Rate r(params.rate);
    while (ros::ok()) {
        ros::Time timestamp = ros::Time::now();

        pose_array.header.stamp = timestamp;

        path_pub.publish(pose_array);

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
