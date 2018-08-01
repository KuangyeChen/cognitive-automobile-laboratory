#pragma once

#include <dynamic_reconfigure/server.h>
#include <ros/forwards.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_listener.h>
#include <Eigen/Eigen>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>

#include "map_creation_ros_tool/MapCreationInterface.h"

namespace map_creation_ros_tool {

    class MapCreation {

        using Interface = MapCreationInterface;
        using ReconfigureConfig = MapCreationConfig;
        using ReconfigureServer = dynamic_reconfigure::Server<ReconfigureConfig>;

    public:
        MapCreation(ros::NodeHandle, ros::NodeHandle);

    private:
        void callbackSubscriber(const sensor_msgs::PointCloud2::ConstPtr &cloud);
        void reconfigureRequest(const ReconfigureConfig&, uint32_t);
        bool getVehiclePosition(Eigen::Affine3d&);
        Interface interface_;
        ReconfigureServer reconfigureServer_;

        ros::Subscriber subscriber_;
        ros::Publisher publisher_;
        nav_msgs::OccupancyGrid map_;
        bool activate_;

        tf2_ros::Buffer tfBuffer_;
        tf2_ros::TransformListener tfListener_;
        tf::TransformListener list_;
        tf2_ros::TransformBroadcaster tfBroadcaster_;
    };
} // namespace map_creation_ros_tool