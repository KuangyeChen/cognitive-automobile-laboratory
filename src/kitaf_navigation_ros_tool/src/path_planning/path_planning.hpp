#pragma once

#include <dynamic_reconfigure/server.h>
#include <ros/forwards.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Int8.h>
#include <kitaf_utils_ros/SwitchToExit.h>

#include "kitaf_navigation_ros_tool/PathPlanningInterface.h"

namespace kitaf_navigation_ros_tool {

class PathPlanning {

    using Interface = PathPlanningInterface;
    using ReconfigureConfig = PathPlanningConfig;
    using ReconfigureServer = dynamic_reconfigure::Server<ReconfigureConfig>;

public:
    PathPlanning(ros::NodeHandle, ros::NodeHandle);

private:
    void initializePath(const std::string&);
    void callbackSubscriber(const nav_msgs::OccupancyGrid::ConstPtr&);
    void updatePath(const ros::TimerEvent&);
    void publishPath(const ros::TimerEvent&);
    void taskCallback(const std_msgs::Int8::ConstPtr&);
    double getPotential(const double);
    void reconfigureRequest(const ReconfigureConfig&, uint32_t);

    Interface interface_;
    ReconfigureServer reconfigureServer_;

    ros::Publisher publisher_;
    ros::Subscriber subscriber_, task_subscriber_;
    ros::Timer update_timer_, publish_timer_;

    double theta_;
    bool activate_;
    nav_msgs::Path path_;
    nav_msgs::OccupancyGrid map_;

    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    tf2_ros::TransformBroadcaster tfBroadcaster_;
};
} // namespace kitaf_navigation_ros_tool
