#pragma once

#include <vector>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <nav_msgs/Path.h>
#include <Eigen/Eigen>
#include <tf2_eigen/tf2_eigen.h>

namespace kitaf_lateral_control_ros_tool {


bool pathFromBag(const std::string& file_name, const std::string& topic,
				 std::vector<Eigen::Affine3d>& path) {

    rosbag::Bag bag(file_name);
    std::vector<std::string> topics{topic};
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    const nav_msgs::Path::ConstPtr path_record = (*view.begin()).instantiate<nav_msgs::Path>();

    path.clear();
    path.reserve(path_record->poses.size());
    std::cout<<"path size"<<path_record->poses.size()<<std::endl;
    for (const auto& pose_stamped : path_record->poses) {
        Eigen::Affine3d pose;
        tf2::fromMsg(pose_stamped.pose, pose);
        path.push_back(pose);
    }

    bag.close();
    return true;
}

}
