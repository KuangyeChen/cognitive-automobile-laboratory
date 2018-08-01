#include "disp2pointcloud.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <rosinterface_handler/utilities.hpp>

using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;

namespace image_preproc_ros_tool {

Disp2pointcloud::Disp2pointcloud(ros::NodeHandle nh_public, ros::NodeHandle nh_private)
        : it_{nh_private}, params_{nh_private}, reconfigureServer_{nh_private} {

    /**
     * Initialization
     */
    rosinterface_handler::setLoggerLevel(nh_private);
    params_.fromParamServer();

    /**
     * Publishers & subscriber
     */

    info_sub_.subscribe(nh_private, params_.info_topic, params_.msg_queue_size);
    disp_sub_.subscribe(nh_private, params_.disp_topic, params_.msg_queue_size);

    ros::SubscriberStatusCallback connect_cb =
        boost::bind(&Disp2pointcloud::subscribeCallback, this);
    cloud_pub_ = nh_private.advertise<sensor_msgs::PointCloud2>(
        params_.pointcloud_topic, params_.msg_queue_size, connect_cb, connect_cb);


    sync_ = std::make_shared<Sync>(img_sub_, disp_sub_, info_sub_, params_.msg_queue_size);
    sync_->registerCallback(boost::bind(&Disp2pointcloud::callbackImage, this, _1, _2, _3));


    reconfigureServer_.setCallback(boost::bind(&Disp2pointcloud::reconfigureRequest, this, _1, _2));

    if (!params_.mask.empty()) {
        mask_ = cv::imread(params_.mask, CV_LOAD_IMAGE_GRAYSCALE);
    }

    if (!params_.remap_in.empty()) {
        doRemap_ = true;
        remap_ = cv::Mat::ones(1, 256, CV_8U) * params_.invalid_value;
        if (params_.remap_in.size() != params_.remap_out.size()) {
            throw std::runtime_error("Remap lookup table has invalid input->output size!");
        }
        if (params_.invalid_value < 0) {
            throw std::runtime_error(
                "Value for invalid fields can not be negative when remap mode is enabled!");
        }
        for (size_t i = 0; i < params_.remap_in.size(); i++) {
            remap_.at<uchar>(0, params_.remap_in[i]) = params_.remap_out[i];
        }
    }
    subscribeCallback();
    rosinterface_handler::showNodeInfo();
}

void Disp2pointcloud::callbackImage(const Disp2pointcloud::Img::ConstPtr& img,
                                    const stereo_msgs::DisparityImage::ConstPtr& disp,
                                    const sensor_msgs::CameraInfo::ConstPtr& info) {
    ROS_DEBUG_STREAM("Image callback");
    cv_bridge::CvImageConstPtr imgBridge = cv_bridge::toCvShare(img);
    cv_bridge::CvImageConstPtr dispBridge = cv_bridge::toCvCopy(disp->image);

    model_.fromCameraInfo(info);

    cv::Mat imgMat;
    if (imgBridge->image.channels() > 1) {
        cv::cvtColor(imgBridge->image, imgMat, CV_BGR2GRAY);
    } else
        imgMat = imgBridge->image.clone();

    if (imgMat.type() != CV_8U) {
        imgMat.convertTo(imgMat, CV_8U);
    }

    if(doRemap_) {
        cv::LUT(imgMat, remap_, imgMat);
    }

    // check input data
    if (dispBridge->image.type() != CV_32F) {
        ROS_ERROR_STREAM_THROTTLE(5, "Disparity image has not float type! ignoring image!");
        return;
    }
    if (!imgMat.isContinuous() || !dispBridge->image.isContinuous()) {
        ROS_ERROR_STREAM_THROTTLE(5, "Received non-continuous image. Ignoring");
        return;
    }
    if (imgMat.size() != dispBridge->image.size()) {
        ROS_ERROR_STREAM_THROTTLE(5, "Image and Disparity image have different sizes!");
        return;
    }
    if (mask_.data && (mask_.size() != imgMat.size())) {
        ROS_ERROR_STREAM("Image and mask have different sizes. Ignoring mask!");
        mask_ = cv::Mat();
    }

    Cloud::Ptr cloud{boost::make_shared<Cloud>(imgMat.cols, imgMat.rows)};
    cloud->header = pcl_conversions::toPCL(info->header);
    const auto f = params_.focal_length > 0 ? params_.focal_length : disp->f;
    const auto cx = model_.cx();
    const auto cy = model_.cy();
    const auto base = params_.base_width > 0 ? params_.base_width : disp->T;
    auto pDisp = dispBridge->image.ptr<float>();
    auto pCloud = cloud->points.data();
    auto pImg = imgMat.ptr<uchar>();
    auto pMask = mask_.ptr<uchar>();
    for (auto y = 0; y < imgMat.rows; y++) {
        for (auto x = 0; x < imgMat.cols; x++, pDisp++, pCloud++, pImg++) {
            const auto& d = *pDisp;
            const auto i = *pImg;
            if (isValid(i) && d >= disp->min_disparity &&
                (d <= disp->max_disparity || disp->max_disparity == 0) && (!pMask || *pMask > 0)) {
                const float scale{static_cast<float>(base / d)};
                pCloud->x = (x - cx) * scale;
                pCloud->y = (y - cy) * scale;
                pCloud->z = scale * f;
                pCloud->intensity = static_cast<float>(i) / 255.;
            } else {
                pCloud->x = 0;
                pCloud->y = 0;
                pCloud->z = 120;
                pCloud->intensity = -1;
            }
            if (pMask)
                pMask++;
        }
    }
    cloud_pub_.publish(cloud);
}

/**
  * This callback is called at startup or whenever a change was made in the
 * dynamic_reconfigure window
*/
void Disp2pointcloud::reconfigureRequest(const Config& config, uint32_t level) {
    params_.fromConfig(config);
}

void Disp2pointcloud::subscribeCallback() {
    if (cloud_pub_.getNumSubscribers() > 0) {
        ROS_DEBUG_STREAM("Subscribing");
        img_sub_.subscribe(it_, params_.image_topic, params_.msg_queue_size);
        info_sub_.subscribe();
        disp_sub_.subscribe();
    } else {
        ROS_DEBUG_STREAM("Unsubscribing");
        img_sub_.unsubscribe();
        info_sub_.unsubscribe();
        disp_sub_.unsubscribe();
    }
}

bool Disp2pointcloud::isValid(unsigned char pix) {
    return pix != params_.invalid_value;
}
} // namespace image_preproc_ros_tool
