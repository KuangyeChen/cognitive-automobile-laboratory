#pragma once

#include <Eigen/Eigen>
#include <dynamic_reconfigure/server.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <ros/timer.h>
#include <tf2_ros/transform_listener.h>

#include "path_provider_ros_tool/PathProviderParameters.h"

namespace path_provider_ros_tool {

class PathProvider {

    using Parameters = PathProviderParameters;
    using ReconfigureServer = dynamic_reconfigure::Server<Parameters::Config>;

public:
    PathProvider(ros::NodeHandle = ros::NodeHandle(), ros::NodeHandle = ros::NodeHandle("~"));

private:
    /**
     * This method is called at a fixed rate by the timer
     * @param TimerEvent structure
     */
    void callbackTimer(const ros::TimerEvent&);

    ros::Timer timer_;
    Parameters params_;                      ///< Keeps all relevant parameters, such as topic names, timer rate, etc.
    ReconfigureServer reconfigure_server_;   ///< Enables us to modify parameters during run-time
    tf2_ros::Buffer tf_buffer_;              ///< Buffers the transformation written to tf
    tf2_ros::TransformListener tf_listener_; ///< Manages tf_buffer object
    ros::Publisher publisher_path_;          ///< Path publisher

    std::map<double, Eigen::Affine3d> poses_;
    nav_msgs::Path::Ptr path_{new nav_msgs::Path};
};
}
