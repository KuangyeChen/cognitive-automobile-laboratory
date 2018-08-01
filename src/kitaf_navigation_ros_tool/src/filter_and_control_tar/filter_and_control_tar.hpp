#pragma once

#include <dynamic_reconfigure/server.h>
#include <ros/forwards.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/Path.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>


#include <image_transport/camera_publisher.h>
#include <image_transport/camera_subscriber.h>
#include <image_transport/image_transport.h>
#include <iostream>



#include "kitaf_navigation_ros_tool/FilterAndControlTarInterface.h"

namespace kitaf_navigation_ros_tool {

class FilterAndControlTar {

    using Interface = FilterAndControlTarInterface;
    using ReconfigureConfig = FilterAndControlTarConfig;
    using ReconfigureServer = dynamic_reconfigure::Server<ReconfigureConfig>;

    using Msg = std_msgs::Header;

public:
    FilterAndControlTar(ros::NodeHandle, ros::NodeHandle);

private:
    //void pathcallbackSubscriber(const nav_msgs::Path msg);
    void pathcallbackSubscriber (const nav_msgs::Path::ConstPtr& msg);
    void callbackSubscriber(const Msg::ConstPtr& msg);
    void loopCallBack(const ros::TimerEvent& timer_event);
    void posecallbackSubscriber(const geometry_msgs::PoseStamped::ConstPtr&);//,const geometry_msgs::PoseStamped::ConstPtr&);



    void reconfigureRequest(const ReconfigureConfig&, uint32_t);

    double signedAngleBetween(const Eigen::Vector3d& a, const Eigen::Vector3d& b);


    std::vector<Eigen::Affine3d> path_;


    ros::Timer loop_timer_km_;
    ros::Publisher km_pose_publisher;
    ros::Subscriber path_subscriber;

    ros::Subscriber pose_subscriber;

    //Eigen::Vector3d object_position_previous;
    Eigen::Vector3d object_position_present;

    double dt;
    int n; //summe von time step
    int m;

    Eigen::Vector4d X_state_akt_m;  //mittelwert von X_state_akt
    Eigen::Vector4d X_state_previous_m;//mittelwert von X_state_previous
    Eigen::Vector4d X_t; //aktuelle Schätzung


    Eigen::Vector3d object_position_km; //nach KM gefilterte objeckt positon

    Eigen::Vector2d y_t; //aktuelle Messung (Subscribe from kitaf_detector)
    Eigen::Vector2d y_t_pre;//

    Eigen::Matrix4d P_previous;//kovarianzmatrix for previous position
    Eigen::Matrix4d P_akt_m;
    Eigen::Matrix4d P_akt; //Kovarianzmatrix P for current postion
    Eigen::Matrix4d P_0;//init value for Kovarianz Matrix P

    Eigen::Matrix<double , 4,2> K_t;  //Verstärkersmatrix K

    //unchanged parameter:
    Eigen::Matrix4d Q_1;
    Eigen::Matrix2d R; //Messrauschkovarianzmatrix
    Eigen::Matrix<double, 2,4> H;
    Eigen::Matrix4d I;


    //changed parameter:
    Eigen::Matrix4d A;


    //for Output:
	geometry_msgs::PoseStamped km_position_forPub;
    Eigen::Vector3d pose_object;
    bool poseReceiv;


    // doesn't used parameter in KM
    Eigen::Vector2d W_previous;
    Eigen::Matrix<double , 4,2> C;
    Eigen::Matrix2d E;

    //Eigen::Matrix4d Q;//Prozessrauschkovarianzmatrix
    Eigen::Vector2d G; //G ist 4x1, es wird in Q eingesetzt
    Eigen::Matrix2d Q;

    Eigen::Vector4d X_t_vor;//previous


    Interface interface_;
    ReconfigureServer reconfigureServer_;

    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    tf2_ros::TransformBroadcaster tfBroadcaster_;
};
} // namespace kitaf_navigation_ros_tool
