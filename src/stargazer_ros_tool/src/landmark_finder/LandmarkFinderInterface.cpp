#include "LandmarkFinderInterface.h"

#include "../StargazerConversionMethods.h"
#include "../ros_utils.h"

using namespace stargazer_ros_tool;

LandmarkFinderInterface::LandmarkFinderInterface(ros::NodeHandle nh_public,
                                                 ros::NodeHandle nh_private)
        : img_trans{nh_public}, server{nh_private} {

    params_.fromNodeHandle(nh_private);
    landmarkFinder = std::make_unique<stargazer::LandmarkFinder>(params_.stargazer_config);
    server.setCallback(boost::bind(&LandmarkFinderInterface::reconfigureCallback, this, _1, _2));
    lm_pub = nh_private.advertise<stargazer_ros_tool::LandmarkArray>(params_.landmark_topic, 1);
    img_sub = img_trans.subscribe(
        params_.undistorted_image_topic, 1, &LandmarkFinderInterface::imgCallback, this);
    debugVisualizer_.SetWaitTime(10);

    if (params_.cfg.debug_mode)
        showNodeInfo();
}

void LandmarkFinderInterface::imgCallback(const sensor_msgs::ImageConstPtr& msg) {

    cv_bridge::CvImagePtr cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);

    std::vector<stargazer::ImgLandmark> detected_img_landmarks;
    landmarkFinder->DetectLandmarks(cvPtr->image, detected_img_landmarks);

    // Convert
    stargazer_ros_tool::LandmarkArray landmarksMessage =
        convert2LandmarkMsg(detected_img_landmarks, msg->header);
    lm_pub.publish(landmarksMessage);

    //  Visualize
    if (!params_.cfg.debug_mode)
        return;

    // Invert images
    cv::bitwise_not(landmarkFinder->grayImage_, landmarkFinder->grayImage_);
    cv::bitwise_not(landmarkFinder->filteredImage_, landmarkFinder->filteredImage_);

    // Show images
    debugVisualizer_.ShowImage(landmarkFinder->grayImage_, "Gray Image");
    debugVisualizer_.ShowImage(landmarkFinder->filteredImage_, "Filtered Image");

    // Show detections
    auto point_img = debugVisualizer_.ShowPoints(landmarkFinder->filteredImage_,
                                                 landmarkFinder->clusteredPixels_);
    auto cluster_img = debugVisualizer_.ShowClusters(landmarkFinder->filteredImage_,
                                                     landmarkFinder->clusteredPoints_);

    // Show landmarks
    cv::Mat temp;
    cvtColor(landmarkFinder->grayImage_, temp, CV_GRAY2BGR);
    debugVisualizer_.DrawLandmarks(temp, detected_img_landmarks);
    debugVisualizer_.ShowImage(temp, "Detected Landmarks");
}

void LandmarkFinderInterface::reconfigureCallback(LandmarkFinderConfig& config,
                                                  const uint32_t& level) {

    params_.fromConfig(config, level);

    landmarkFinder->tight_filter_size = static_cast<uint32_t>(params_.cfg.tight_filter_size);
    landmarkFinder->wide_filter_size = static_cast<uint32_t>(params_.cfg.wide_filter_size);
    landmarkFinder->threshold = static_cast<uint8_t>(params_.cfg.threshold);
    landmarkFinder->maxRadiusForCluster = params_.cfg.maxRadiusForCluster;
    landmarkFinder->maxRadiusForPixelCluster = params_.cfg.maxRadiusForPixelCluster;
    landmarkFinder->maxPixelForCluster = static_cast<uint16_t>(params_.cfg.maxPixelForCluster);
    landmarkFinder->minPixelForCluster = static_cast<uint16_t>(params_.cfg.minPixelForCluster);
    landmarkFinder->maxPointsPerLandmark = static_cast<uint16_t>(params_.cfg.maxPointsPerLandmark);
    landmarkFinder->minPointsPerLandmark = static_cast<uint16_t>(params_.cfg.minPointsPerLandmark);
}
