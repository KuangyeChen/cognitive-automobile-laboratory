#pragma once

#include <array>
#include <dynamic_reconfigure/server.h>
#include <mrt_image_geometry_ros/pinhole_camera_model.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <opencv2/core/version.hpp>

#if CV_MAJOR_VERSION == 2
#include <opencv2/core/core.hpp>
#elif CV_MAJOR_VERSION == 3
#include <opencv2/core.hpp>
#endif

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <stereo_msgs/DisparityImage.h>

#include "image_preproc_ros_tool/Disp2pointcloudInterface.h"

namespace image_preproc_ros_tool {

class Disp2pointcloud {

    using Parameters = Disp2pointcloudInterface;
    using Config = Disp2pointcloudConfig;
    using ReconfigureServer = dynamic_reconfigure::Server<Config>;

    using Img = sensor_msgs::Image;
    using Info = sensor_msgs::CameraInfo;
    using Disp = stereo_msgs::DisparityImage;
    using Sync = message_filters::TimeSynchronizer<Img, Disp, Info>;
    using Sync_ptr = std::shared_ptr<Sync>;

public:
    Disp2pointcloud(ros::NodeHandle, ros::NodeHandle);

private:
    void callbackImage(const Img::ConstPtr& img,
                       const Disp::ConstPtr& disp,
                       const Info::ConstPtr& info);
    void reconfigureRequest(const Config&, uint32_t);
    void subscribeCallback();


    bool isValid(unsigned char pix);

    ros::Publisher cloud_pub_;
    image_transport::SubscriberFilter img_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub_;
    message_filters::Subscriber<Disp> disp_sub_;
    image_transport::ImageTransport it_;
    image_geometry::PinholeCameraModel model_;
    Sync_ptr sync_;
    Parameters params_;
    ReconfigureServer reconfigureServer_;
    cv::Mat mask_;
    cv::Mat remap_;

    bool doRemap_{false};
};
} // namespace image_preproc_ros_tool
