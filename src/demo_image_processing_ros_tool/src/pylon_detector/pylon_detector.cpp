#include "pylon_detector.hpp"

#include <cv_bridge/cv_bridge.h>
#include <motor_interface_ros_tool/Activation.h>
#include <motor_interface_ros_tool/MotorCommand.h>
#include <motor_interface_ros_tool/ServoCommand.h>
#include <utils_ros/ros_console.hpp>
#include <opencv2/core/version.hpp>

#if CV_MAJOR_VERSION == 2
#include <opencv2/imgproc/imgproc.hpp>
#elif CV_MAJOR_VERSION == 3
#include <opencv2/imgproc.hpp>
#endif

namespace demo_image_processing_ros_tool {

PylonDetector::PylonDetector(ros::NodeHandle nh_public, ros::NodeHandle nh_private)
        : params_{nh_private}, reconfigure_server_{nh_private}, image_transport_{nh_public} {

    /**
     * Initialization
     */
    utils_ros::setLoggerLevel(nh_private);
    params_.fromParamServer();

    /**
     * Publisher and subscriber setup
     */
    publisher_ = image_transport_.advertiseCamera(params_.topic_publisher, params_.msg_queue_size);
    motor_command_publisher_ = nh_private.advertise<motor_interface_ros_tool::MotorCommand>(params_.motor_command_topic, params_.msg_queue_size);
    servo_command_publisher_ = nh_private.advertise<motor_interface_ros_tool::ServoCommand>(params_.servo_command_topic, params_.msg_queue_size);
    subscriber_ = image_transport_.subscribeCamera(
        params_.topic_subscriber,
        params_.msg_queue_size,
        boost::bind(&PylonDetector::callbackSubscriber, this, _1, _2));

    /**
     * Set up dynamic reconfiguration
     */
    reconfigure_server_.setCallback(boost::bind(&Parameters::fromConfig, &params_, _1, _2));

    /**
     * Setup service client to motor control
     */
    service_client_ = nh_public.serviceClient<motor_interface_ros_tool::Activation>("/motor_interface/activate");

    /**
     * Display a summary after everything is set up
     */
    utils_ros::showNodeInfo();
}

void PylonDetector::callbackSubscriber(const sensor_msgs::Image::ConstPtr& msg,
                                       const sensor_msgs::CameraInfo::ConstPtr& camera_info) {

    /**
     * Convert ROS message using the cv_bridge
     */
    namespace enc = sensor_msgs::image_encodings;
    if (!enc::isColor(msg->encoding)) {
        ROS_ERROR_STREAM_THROTTLE(1, "Input is no color image");
        return;
    }

    cv_bridge::CvImage::ConstPtr cv_in;
    try {
        cv_in = cv_bridge::toCvShare(msg, enc::BGR8);
    } catch (const cv_bridge::Exception& e) {
        ROS_ERROR_STREAM(e.what());
        return;
    }

    /**
     * Perform image segmentation
     * 1. Convert input image to HSV color space
     * 2. Find all pixel values within certain range
     * 3. Apply median blur for robustness
     * 4. Count segmented pixels
     */
    cv::Mat img_hsv;
    cv::cvtColor(cv_in->image, img_hsv, cv::COLOR_BGR2HSV);
    const cv_bridge::CvImage::Ptr cv_out{new cv_bridge::CvImage(msg->header, enc::MONO8)};
    cv::inRange(img_hsv,
                cv::Scalar(params_.h_min, params_.s_min, params_.v_min),
                cv::Scalar(params_.h_max, params_.s_max, params_.v_max),
                cv_out->image);
    cv::medianBlur(cv_out->image, cv_out->image, 2 * params_.median_blur_kernel_size - 1);
    const int count{cv::countNonZero(cv_out->image == 255)};
    ROS_DEBUG_STREAM("Number of segmented pixels: " << count);

    const bool activate = count < params_.threshold;

    /**
     * Publish segmented image
     */
    publisher_.publish(cv_out->toImageMsg(), camera_info);

    if (activated_ == activate) ///< Nothing has changed
        return;

    activated_ = activate;
    ROS_DEBUG_STREAM("Scene changed. activated_: " << activated_);
    motor_interface_ros_tool::Activation activation;
    activation.request.activate = activated_;
    if (service_client_.call(activation))
        ROS_INFO_STREAM("Motor interface called successfully");
    else
        ROS_ERROR_STREAM("Could not connect to motor interface");
    
    motor_interface_ros_tool::MotorCommand motor_command;
    motor_command.header.stamp = msg->header.stamp;
    motor_command.velocity = activated_ ? 0.2 : 0.0;
    motor_command_publisher_.publish(motor_command);

    motor_interface_ros_tool::ServoCommand servo_command;
    servo_command.header.stamp = msg->header.stamp;
    servo_command.steering_angle = 0.0;
    servo_command_publisher_.publish(servo_command);
}
}
