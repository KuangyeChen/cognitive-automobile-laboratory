#pragma once

#include <dynamic_reconfigure/server.h>
#include <ros/forwards.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <std_msgs/Int8.h>
#include <nav_msgs/Path.h>
#include <Eigen/Eigen>

#include <kitaf_utils_ros/detection_signals.hpp>

#include "kitaf_navigation_ros_tool/KitafPathProviderInterface.h"

namespace kitaf_navigation_ros_tool {

class KitafPathProvider {

    using Interface = KitafPathProviderInterface;
    using ReconfigureConfig = KitafPathProviderConfig;
    using ReconfigureServer = dynamic_reconfigure::Server<ReconfigureConfig>;


public:
    KitafPathProvider(ros::NodeHandle, ros::NodeHandle);

private:
    void signCallback(const std_msgs::Int8::ConstPtr&);
    void pathPublishCallback(const ros::TimerEvent&);
    void pathSwitchCallback(const ros::TimerEvent&);
    inline void publishPath();
    void reconfigureRequest(const ReconfigureConfig&, uint32_t);
    void readAllData(const std::string&);
    void setNearestPath();
    bool getVehiclePosition(Eigen::Affine3d&);
    void publishTask(const ros::TimerEvent&);

    Interface interface_;
    ReconfigureServer reconfigureServer_;

    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    tf2_ros::TransformBroadcaster tfBroadcaster_;

    ros::NodeHandlePtr nh_ptr_;
    ros::Timer switch_timer_, publish_timer_, task_pub_timer_;
    ros::Publisher path_publisher_, task_publisher_;
    ros::Subscriber sign_subscriber_;

    kitaf_utils_ros::DetectionSignal sign_;
    nav_msgs::Path::ConstPtr path_;
    Eigen::Vector3d end_point_;
    Eigen::Vector3d center_point_;
    Eigen::Vector3d exit_point_;
    bool ready_watch_, pass_center_, near_center_;
    /**
     * Self state representing next intersection.
     * State: A B C D
     * Int:   0 1 2 3
     */
    int32_t state_, task_;

    /**
     * Stored paths and state change.
     *
     *         A     B     C     D
     *         f l r f l r f l r f  l  r
     * vector: 0 1 2 3 4 5 6 7 8 9 10 11
     * f - forward
     * l - left
     * r - right
     */
    const std::vector<int32_t> state_change_{1, 0, 2,
                                             0, 3, 1,
                                             3, 2, 0,
                                             2, 1, 3};
    const std::vector<std::string> all_path_name_{"a_forward", "a_left", "a_right",
                                                  "b_forward", "b_left", "b_right",
                                                  "c_forward", "c_left", "c_right",
                                                  "d_forward", "d_left", "d_right"};
    std::vector<nav_msgs::Path::ConstPtr> all_path_;
    std::vector<Eigen::Vector3d> all_end_point_;
};
} // namespace kitaf_navigation_ros_tool
