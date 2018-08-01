#pragma once

#include <dynamic_reconfigure/server.h>
#include <image_transport/camera_publisher.h>
#include <image_transport/camera_subscriber.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <ros/service_client.h>
#include <sensor_msgs/Image.h>

#include "demo_image_processing_ros_tool/PylonDetectorParameters.h"

namespace demo_image_processing_ros_tool {

class PylonDetector {

    using Parameters = PylonDetectorParameters;
    using ReconfigureServer = dynamic_reconfigure::Server<Parameters::Config>;

public:
    PylonDetector(ros::NodeHandle = ros::NodeHandle(), ros::NodeHandle = ros::NodeHandle("~"));

private:
    /**
     * This callback is called at startup or whenever a change was made in the dynamic_reconfigure
     * window
     * @param Received image
     */
    void callbackSubscriber(const sensor_msgs::Image::ConstPtr&,
                            const sensor_msgs::CameraInfoConstPtr&);

    Parameters params_; ///< Keeps all the parameters defined in "cfg/PylonDetector.mrtcfg"
    ReconfigureServer reconfigure_server_; ///< Enables us to change parameters during run-time
    image_transport::ImageTransport image_transport_; ///< Recommended image handle mechanism
    image_transport::CameraPublisher publisher_;      ///< Publishes the segmented image
    image_transport::CameraSubscriber subscriber_;    ///< Subscribes to the color image
    ros::Publisher motor_command_publisher_; ///< publishes commands to the motor
    ros::Publisher servo_command_publisher_;
    ros::ServiceClient service_client_; ///< Lets us send a request to the motor interface
    bool activated_{false};             ///< Keeps the last state
};
}
