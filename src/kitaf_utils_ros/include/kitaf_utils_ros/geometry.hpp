//
// Created by kal3-1 on 15.06.18.
//

#pragma once

#include <Eigen/Eigen>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>

namespace kitaf_utils_ros {

bool getTransform(const tf2_ros::Buffer& tfBuffer, const std::string& target_frame, const std::string& source_frame,
                 Eigen::Affine3d& transform, const ros::Time time=ros::Time(0));

double cross2d(const Eigen::Vector3d&, const Eigen::Vector3d&);

}