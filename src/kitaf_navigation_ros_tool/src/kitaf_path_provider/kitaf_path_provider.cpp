#include <fstream>
#include <tf2_eigen/tf2_eigen.h>
#include <std_msgs/Int8.h>

#include "kitaf_path_provider.hpp"

namespace kitaf_navigation_ros_tool {

using namespace kitaf_utils_ros;

KitafPathProvider::KitafPathProvider(ros::NodeHandle nhPublic, ros::NodeHandle nhPrivate)
        : interface_{nhPrivate}, reconfigureServer_{nhPrivate}, tfListener_{tfBuffer_}, nh_ptr_{&nhPrivate},
          sign_{SEE_NO_TRAFFIC_SIGN}, center_point_{0.0, 0.0, 0.0}, ready_watch_{false}, pass_center_{false},
          near_center_{false}, state_{-1}, task_{0}, all_path_{}, all_end_point_{} {

    /**
     * Initialization
     */
    interface_.fromParamServer();

    reconfigureServer_.setCallback(boost::bind(&KitafPathProvider::reconfigureRequest, this, _1, _2));
    sign_subscriber_ = nhPrivate.subscribe(interface_.sign_topic,
                                           interface_.sign_queue_size,
                                           &KitafPathProvider::signCallback,
                                           this);
    path_publisher_ = nhPrivate.advertise<nav_msgs::Path>(interface_.path_topic,
                                                          interface_.path_queue_size);
    task_publisher_ = nhPrivate.advertise<std_msgs::Int8>("/current_task",
                                                          interface_.path_queue_size);

    std::string data_path;
    nhPrivate.getParam("data_path", data_path);
    readAllData(data_path);
    setNearestPath();

    switch_timer_ = nhPrivate.createTimer(ros::Rate(interface_.switch_timer_rate),
                                          &KitafPathProvider::pathSwitchCallback,
                                          this);
    publish_timer_ = nhPrivate.createTimer(ros::Rate(interface_.publish_timer_rate),
                                           &KitafPathProvider::pathPublishCallback,
                                           this);
    task_pub_timer_ = nhPrivate.createTimer(ros::Rate(30.0),
                                            &KitafPathProvider::publishTask,
                                            this);
    rosinterface_handler::showNodeInfo();


}

void KitafPathProvider::readAllData(const std::string& store_path) {
    for (const auto& path : all_path_name_){
        std::ifstream path_file(store_path + path + ".txt");
        nav_msgs::Path::Ptr new_path{new nav_msgs::Path};
        new_path -> header.frame_id = interface_.map_frame_id;

        geometry_msgs::PoseStamped pose;
        pose.header = new_path -> header;

        double x{-1}, y{-1};
        while(path_file >> x && path_file >> y) {
            pose.pose.position.x = x;
            pose.pose.position.y = y;
            new_path -> poses.emplace_back(pose);
        }
        path_file.close();

        all_path_.emplace_back(new_path);
        all_end_point_.emplace_back(x, y, 0);

        ROS_INFO_STREAM(path << " loaded. Number of points: " << new_path -> poses.size());
    }

    std::ifstream center_file(store_path + "center.txt");
    center_file >> center_point_[0] >> center_point_[1];
    center_file.close();
    ROS_INFO_STREAM("Center point loaded: " << center_point_[0] << " " << center_point_[1]);
    std::ifstream exit_file(store_path + "exit.txt");
    exit_file >> exit_point_[0] >> exit_point_[1];
    exit_file.close();
    ROS_INFO_STREAM("Exit point loaded: " << exit_point_[0] << " " << exit_point_[1]);
}

void KitafPathProvider::setNearestPath() {
    Eigen::Affine3d vehicle_pose;
    while(!getVehiclePosition(vehicle_pose)) {
        ROS_WARN_STREAM("Retry in 1 second.");
        ros::Duration(1).sleep();
    }
    Eigen::Vector3d vehicle_position = vehicle_pose.translation();
    const Eigen::Vector3d vehicle_direction = vehicle_pose.rotation() * Eigen::Vector3d::UnitX();

    auto min_distance = DBL_MAX;
    for (auto iter = all_path_.begin(); iter != all_path_.end(); ++iter) {
        const auto& min_iter = std::min_element((*iter) -> poses.begin(), (*iter) -> poses.end(),
                [&vehicle_position](const geometry_msgs::PoseStamped& lhs, const geometry_msgs::PoseStamped& rhs) {
                    Eigen::Vector3d lhs_vec(lhs.pose.position.x, lhs.pose.position.y, 0.0);
                    Eigen::Vector3d rhs_vec(rhs.pose.position.x, rhs.pose.position.y, 0.0);
                    return (lhs_vec - vehicle_position).norm() < (rhs_vec - vehicle_position).norm();
        });

        /**
         * Use end part of path.
         */
        auto path_center_iter = (*iter) -> poses.begin();
        for (; path_center_iter != (*iter) -> poses.end(); ++path_center_iter) {
            Eigen::Vector3d point_position(path_center_iter -> pose.position.x,
                                           path_center_iter -> pose.position.y,
                                           0.0);
            if ((point_position - center_point_).norm() < 2.0) { break; }
        }
        if (min_iter < path_center_iter ) {
            continue;
        }

        /**
         * Only consider path with right direction.
         */
        auto now_iter = min_iter, next_iter = min_iter + 1;
        if (next_iter == ((*iter) -> poses).end()) {
            next_iter = min_iter;
            now_iter = min_iter - 1;
        }
        const Eigen::Vector3d path_direction(next_iter -> pose.position.x - now_iter -> pose.position.x,
                                             next_iter -> pose.position.y - now_iter -> pose.position.y,
                                             0.0);
        if (vehicle_direction.dot(path_direction) < 0) {
            continue;
        }

        /**
         * Find suitable path.
         */
        Eigen::Vector3d min_point(min_iter -> pose.position.x, min_iter -> pose.position.y, 0.0);
        const double distance_path = (min_point - vehicle_position).norm();
        if (distance_path < min_distance) {
            min_distance = distance_path;
            state_ = static_cast<int32_t>(iter - all_path_.begin());
        }
    }

    path_ = all_path_[state_];
    end_point_ = all_end_point_[state_];
    ROS_INFO_STREAM("Initialized with path " << all_path_name_[state_]);

    state_ = state_change_[state_];
    ready_watch_ = false;
    pass_center_ = true;
    near_center_ = true;
}

void KitafPathProvider::signCallback(const std_msgs::Int8::ConstPtr& msg) {
    sign_ = static_cast<DetectionSignal>(msg->data);
    if (sign_ < MIN_DEFINE || sign_ > MAX_DEFINE || sign_ == STOP_SIGN) {
        sign_ = SEE_NO_TRAFFIC_SIGN;
    }
}

inline void KitafPathProvider::publishPath() {
    path_publisher_.publish(path_);
}

void KitafPathProvider::pathPublishCallback(const ros::TimerEvent& event) {
    if (task_ == 1) { return; }

    publishPath();
}

void KitafPathProvider::publishTask(const ros::TimerEvent& event) {
    std_msgs::Int8 msg;
    msg.data = static_cast<int8_t>(task_);
    task_publisher_.publish(msg);
}

void KitafPathProvider::pathSwitchCallback(const ros::TimerEvent& event) {
    if (task_ == 1) { return; }

    Eigen::Affine3d vehicle_pose;
    if(!getVehiclePosition(vehicle_pose)) {
        return;
    }
    Eigen::Vector3d vehicle_position = vehicle_pose.translation();

    const double distance2exit = (vehicle_position - exit_point_).norm();
    const double distance2end = (vehicle_position - end_point_).norm();
    const double distance2center = (vehicle_position - center_point_).norm();

    ROS_DEBUG_STREAM_THROTTLE(3, "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n"
            << "Car state : " << char(state_ + 'A')      << "\n"
            << "    near_center: " << near_center_
            << "    pass_center: " << pass_center_
            << "    ready_watch: " << ready_watch_       << "\n"
            << "Distance to end point: " << distance2end << "\n"
            << "Distance to center: " << distance2center << "\n"
            << "Distance to exit: " << distance2exit);

    if (distance2exit < interface_.exit_watch_radius && state_ == 2 && sign_ == RIGHT_SIGN) {
        ROS_INFO_STREAM("Exit to final task.");
        task_ = 1;
    }

    if (!near_center_) {
        near_center_ = distance2center < interface_.start_watch_radius;
        return;
    }
    if (!pass_center_) {
        pass_center_ = distance2center > interface_.start_watch_radius;
        return;
    }
    if (!ready_watch_) {
        ready_watch_ = distance2center < interface_.start_watch_radius;
        return;
    }

    switch (sign_) {
        case LEFT_SIGN:
            path_ = all_path_[state_ * 3 + 1];
            end_point_ = all_end_point_[state_ * 3 + 1];
            state_ = state_change_[state_ * 3 + 1];
            ROS_INFO_STREAM("Turn Left.");
            break;
        case RIGHT_SIGN:
            path_ = all_path_[state_ * 3 + 2];
            end_point_ = all_end_point_[state_ * 3 + 2];
            state_ = state_change_[state_ * 3 + 2];
            ROS_INFO_STREAM("Turn Right.");
            break;
        case FORWARD_SIGN:
            path_ = all_path_[state_ * 3];
            end_point_ = all_end_point_[state_ * 3];
            state_ = state_change_[state_ * 3];
            ROS_INFO_STREAM("Go Forward.");
            break;
        default:
            if (distance2end > interface_.keep_forward_radius) { return; }
            path_ = all_path_[state_ * 3];
            end_point_ = all_end_point_[state_ * 3];
            state_ = state_change_[state_ * 3];
            ROS_INFO_STREAM("See No Sign, Go Forward.");
    }

    near_center_ = false;
    pass_center_ = false;
    ready_watch_ = false;
    publishPath();
}

bool KitafPathProvider::getVehiclePosition(Eigen::Affine3d& vehicle_position) {
    try {
        const geometry_msgs::TransformStamped tf_ros =
                tfBuffer_.lookupTransform(interface_.map_frame_id, interface_.vehicle_frame_id, ros::Time(0));
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
void KitafPathProvider::reconfigureRequest(const ReconfigureConfig& config, uint32_t level) {
    interface_.fromConfig(config);
}

} // namespace kitaf_navigation_ros_tool
