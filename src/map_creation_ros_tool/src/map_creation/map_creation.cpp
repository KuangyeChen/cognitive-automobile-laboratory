#include "map_creation.hpp"

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf2_eigen/tf2_eigen.h>

#include <Eigen/Eigen>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>

namespace map_creation_ros_tool {

MapCreation::MapCreation(ros::NodeHandle nhPublic, ros::NodeHandle nhPrivate)
        : interface_{nhPrivate}, reconfigureServer_{nhPrivate}, activate_{true}, tfListener_{tfBuffer_} {

    /**
     * Initialization
     */
    interface_.fromParamServer();


    /**
     * Set up callbacks for subscribers and reconfigure.
     *
     * New subscribers can be created with "add_subscriber" in "cfg/MapCreation.if file.
     * Don't forget to register your callbacks here!
     */
    reconfigureServer_.setCallback(boost::bind(&MapCreation::reconfigureRequest, this, _1, _2));

    subscriber_ = nhPrivate.subscribe("/clean_points/output", 2, &MapCreation::callbackSubscriber,this);
    publisher_ = nhPrivate.advertise<nav_msgs::OccupancyGrid>("/map", 1);

    map_.header.frame_id="world";
    map_.info.height = interface_.map_height / interface_.resolution;
    map_.info.width = interface_.map_width / interface_.resolution;
    map_.info.resolution = interface_.resolution;
    geometry_msgs::Pose pose_map;
    pose_map.position.x = interface_.origin_x;
    pose_map.position.y = interface_.origin_y;
    pose_map.position.z = 0.0;
    pose_map.orientation.x = 0.0;
    pose_map.orientation.y = 0.0;
    pose_map.orientation.z = 0.0;
    pose_map.orientation.w = 0.0;
    map_.info.origin=pose_map;
    map_.data = std::vector<int8_t>(map_.info.width * map_.info.height, 50);

    rosinterface_handler::showNodeInfo();
}

void MapCreation::callbackSubscriber(const sensor_msgs::PointCloud2::ConstPtr &cloud) {
    Eigen::Affine3d vehicle_pose;
    if(!getVehiclePosition(vehicle_pose)) {
        return;
    }
    Eigen::Vector3d vehicle_position = vehicle_pose.translation();
    Eigen::Vector3d start_mapping_point(13.111, 0.708,0);
    const double distance2start = (vehicle_position - start_mapping_point).norm();
    if ( !activate_ && distance2start > 0.4 ) {
        publisher_.publish(map_);
        return;
    }
    else if (!activate_ && distance2start < 0.4) {
        activate_ = true;
    }

    pcl::PointCloud<pcl::PointXYZRGB> puntos;
    pcl::PointCloud<pcl::PointXYZRGB> salida;

    // Process the cloud
    pcl::fromROSMsg(*cloud, puntos);

    if ( !pcl_ros::transformPointCloud("world", puntos, salida, list_) ) {
        ROS_WARN_STREAM("Can not transform point cloud to world frame.");
        return;
    }

    Eigen::MatrixXi update_mask = Eigen::MatrixXi::Zero(map_.info.height, map_.info.width);
    Eigen::MatrixXd update_value = Eigen::MatrixXd::Ones(map_.info.height, map_.info.width) *
                                   (100.0 - interface_.model_confidence);
    for (auto& point : salida.points){
        if (point.r == 0 && point.g == 0 && point.b == 0){
            continue;
        }
        if (point.y < map_.info.origin.position.y || point.x < map_.info.origin.position.x) {
            continue;
        }

        auto index_row = static_cast<size_t>( (point.y - map_.info.origin.position.y) / map_.info.resolution);
        auto index_col = static_cast<size_t>( (point.x - map_.info.origin.position.x) / map_.info.resolution);

        if (index_row >= map_.info.height || index_col >= map_.info.width) {
            continue;
        }

        ++update_mask(index_row, index_col);
        if (point.r > 180 && point.g < 100 && point.z > 0.05 && point.z < 0.4) {
            update_value(index_row, index_col) += interface_.weight_per_point;
            update_value(index_row, index_col) =
                    update_value(index_row, index_col) > 99.0 ? 99.0 : update_value(index_row, index_col);
        }
    }

    for (size_t i = 0; i < map_.info.height; i++)
        for (size_t j = 0; j < map_.info.width; j++) {
            if (update_mask(i, j) < interface_.update_threshold) { continue; }

            auto old_value = static_cast<double>(map_.data[i * map_.info.width + j]);

            double s1 = old_value / (100.0 - old_value);
            double s2 = update_value(i, j) / ( 100.0 - update_value(i, j) );

            double s = s1 * s2;
            auto new_value = static_cast<int8_t>(100 * s / (s + 1));

            new_value = new_value > 99 ? 99 : new_value;
            new_value = new_value < 1 ? 1 : new_value;
            map_.data[i * map_.info.width + j] = new_value;
        }

    publisher_.publish(map_);
}

bool MapCreation::getVehiclePosition(Eigen::Affine3d& vehicle_position) {
    try {
        const geometry_msgs::TransformStamped tf_ros =
                tfBuffer_.lookupTransform("world", "vehicle_front_axle", ros::Time(0));
        vehicle_position = tf2::transformToEigen(tf_ros);
    } catch (const tf2::TransformException& e) {
        ROS_WARN_STREAM("Can not get vehicle position: " << e.what());
        return false;
    }
    return true;
}

/**
  * This callback is called at startup or whenever a change was made in the dynamic_reconfigure window
*/
void MapCreation::reconfigureRequest(const ReconfigureConfig& config, uint32_t level) {
    interface_.fromConfig(config);
}

} // namespace map_creation_ros_tool