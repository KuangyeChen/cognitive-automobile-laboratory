//
// Created by kal3-1 on 15.06.18.
//

#include <tf2_eigen/tf2_eigen.h>

#include "geometry.hpp"

namespace kitaf_utils_ros {

    bool getTransform(const tf2_ros::Buffer& tfBuffer, const std::string& target_frame, const std::string& source_frame,
                     Eigen::Affine3d& transform, const ros::Time time) {
        try {
            const geometry_msgs::TransformStamped tf_ros =
                    tfBuffer.lookupTransform(target_frame, source_frame, time);
            transform = tf2::transformToEigen(tf_ros);
        } catch (const tf2::TransformException& e) {
            ROS_WARN_STREAM("Can not get transform from " << source_frame << " to " << target_frame << "\n" << e.what());
            return false;
        }
        return true;
    }

    double cross2d(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
        return a[0] * b[1] - a[1] * b[0];
    }

}