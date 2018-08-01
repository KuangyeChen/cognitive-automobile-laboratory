#include <ros/ros.h>
#include <ros/service_client.h>
#include <sensor_msgs/SetCameraInfo.h>

#include <stargazer/StargazerConfig.h>
#include "../ros_utils.h"

using namespace stargazer_ros_tool;
using namespace stargazer;

int main(int argc, char** argv) {
    ros::init(argc, argv, "intrinsics_tool");
    ros::NodeHandle nh("~");

    ros::ServiceClient set_camera_info = nh.serviceClient<sensor_msgs::SetCameraInfo>("/camera/set_camera_info");

    if (!set_camera_info.exists()) {
        ROS_ERROR_STREAM(set_camera_info.getService() << " does not exist! "
                                                         "Is the camera node running? Does the remap match?");
        return EXIT_FAILURE;
    }

    std::string stargazer_config_path;
    getParam(nh, "stargazer_config_path", stargazer_config_path);

    landmark_map_t landmarks;
    camera_params_t camera_intrinsics;
    try {
        readConfig(stargazer_config_path, camera_intrinsics, landmarks);
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM(std::string(e.what()));
    }

    sensor_msgs::SetCameraInfo::Request set_camera_info_request;
    bool got_camera_info = false;
    ros::Subscriber camera_info_subscriber = nh.subscribe<sensor_msgs::CameraInfo>(
        "/camera/camera_info", 1, [&](const sensor_msgs::CameraInfo::ConstPtr msg) {
        set_camera_info_request.camera_info = *msg;
        got_camera_info = true;
      });

    ros::Duration(0.5).sleep();
    ros::spinOnce();

    if (!got_camera_info) {
      ROS_ERROR_STREAM("Could not recive old camera_info from '" <<
                       camera_info_subscriber.getTopic() << "' !");
      return EXIT_FAILURE;
    }

    set_camera_info_request.camera_info.K[0] = camera_intrinsics[(int)INTRINSICS::fu];
    set_camera_info_request.camera_info.K[1] = 0.0;
    set_camera_info_request.camera_info.K[2] = camera_intrinsics[(int)INTRINSICS::u0];

    set_camera_info_request.camera_info.K[3] = 0.0;
    set_camera_info_request.camera_info.K[4] = camera_intrinsics[(int)INTRINSICS::fv];
    set_camera_info_request.camera_info.K[5] = camera_intrinsics[(int)INTRINSICS::v0];

    set_camera_info_request.camera_info.K[6] = 0.0;
    set_camera_info_request.camera_info.K[7] = 0.0;
    set_camera_info_request.camera_info.K[8] = 0.0;;

    sensor_msgs::SetCameraInfo::Response set_camera_info_response;
    if (!set_camera_info.call(set_camera_info_request, set_camera_info_response)) {
      ROS_ERROR_STREAM("Calling '" << set_camera_info.getService() << "' failed!");
      return EXIT_FAILURE;
    }

    if (!set_camera_info_response.success) {
      ROS_ERROR_STREAM("Setting camera info failed with status_message: " <<
                       set_camera_info_response.status_message);
      return EXIT_FAILURE;
    }

    ROS_INFO_STREAM("Setting camera info succeeded with status_message: " <<
                    set_camera_info_response.status_message);

    return EXIT_SUCCESS;
}
