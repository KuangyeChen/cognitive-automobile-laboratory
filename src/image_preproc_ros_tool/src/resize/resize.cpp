#include "resize.h"
#include <sensor_msgs/image_encodings.h>

Resize::Resize(ros::NodeHandle& nodeHandle, ros::NodeHandle& privateNodeHandle, const std::string name)
  : params_{privateNodeHandle}, reconfigureServer_{privateNodeHandle}, name_{name} {
  params_.fromParamServer();
  imageTransport_ = std::make_unique<image_transport::ImageTransport>(nodeHandle);

  pub_ = nodeHandle.advertise<sensor_msgs::Image>(params_.output_image, params_.msg_queue_size);
  image_transport::SubscriberStatusCallback connect_cb = boost::bind(&Resize::connectCallback, this);
  pubCamera_ = imageTransport_->advertiseCamera(params_.output_camera_info, params_.msg_queue_size, connect_cb, connect_cb);
  sub_.subscribe(nodeHandle, params_.input_image, params_.msg_queue_size, ros::TransportHints().tcpNoDelay());
  sub_.addPublisher(pub_);
  sub_.registerCallback(boost::bind(&Resize::imageCallback, this, _1));
  reconfigureServer_.setCallback(boost::bind(&Resize::reconfigureRequest, this, _1, _2));
  connectCallback();
}

void Resize::reconfigureRequest(const Config &config, uint32_t level) {
  params_.fromConfig(config);
}

void Resize::connectCallback()
{
  if(pubCamera_.getNumSubscribers() > 0 || pub_.getNumSubscribers() > 0){
    subCamera_ = imageTransport_->subscribeCamera(params_.input_camera_info, params_.msg_queue_size, &Resize::cameraCallback, this);
  }
  else{
    subCamera_.shutdown();
  }
}

void Resize::imageCallback(const sensor_msgs::ImageConstPtr& imgMsg) {
  auto newImgMsg = handleImage(imgMsg);
  pub_.publish(newImgMsg);
}

void Resize::cameraCallback(const sensor_msgs::ImageConstPtr& imgMsg,
                            const sensor_msgs::CameraInfoConstPtr& infoMsg) {
  auto newImgMsg = handleImage(imgMsg);
  auto newInfoMsg = boost::make_shared<sensor_msgs::CameraInfo>();

  /* The CameraInfo message contains binning and ROI fields that should be used
   * if the image is cropped and resized. Unfortunately, the binning parameters
   * are integer so that arbitrary scale factors are not supported.
   * Because of that we will modify the camera parameters here. Only K will be
   * valid after this step. */
  double scaleX = 1.0;
  double scaleY = 1.0;
  if (infoMsg->binning_x > 0) {
    scaleX /= infoMsg->binning_x;
  }
  if (infoMsg->binning_y > 0) {
    scaleY /= infoMsg->binning_y;
  }
  if (params_.new_width >= 0 && params_.new_height >= 0) {
    if (params_.roi_x >= 0 && params_.roi_y >= 0 && params_.roi_width > 0 && params_.roi_height > 0) {
      scaleX *= static_cast<double>(params_.new_width) / static_cast<double>(params_.roi_width);
      scaleY *= static_cast<double>(params_.new_height) / static_cast<double>(params_.roi_height);
    } else {
      scaleX *= static_cast<double>(params_.new_width) / static_cast<double>(imgMsg->width);
      scaleY *= static_cast<double>(params_.new_height) / static_cast<double>(imgMsg->height);
    }
  } else if (params_.scale > 0) {
    scaleX *= params_.scale;
    scaleY *= params_.scale;
  }

  int shiftX = infoMsg->roi.x_offset;
  int shiftY = infoMsg->roi.y_offset;
  
  // limit roi width
  const int roi_width = std::min(params_.roi_width, int(imgMsg->width) - params_.roi_x);
  const int roi_height = std::min(params_.roi_height, int(imgMsg->height) - params_.roi_y);
  if (params_.roi_x >= 0 && params_.roi_y >= 0 && roi_width > 0 && roi_height > 0) {
    shiftX += params_.roi_x;
    shiftY += params_.roi_y;
  }

  newInfoMsg->header = infoMsg->header;
  newInfoMsg->distortion_model = infoMsg->distortion_model;
  newInfoMsg->D = infoMsg->D;
  newInfoMsg->R = infoMsg->R;
  newInfoMsg->height = newImgMsg->height;
  newInfoMsg->width = newImgMsg->width;
  newInfoMsg->K[0] = infoMsg->K[0] * scaleX;
  newInfoMsg->K[3] = infoMsg->K[3] * scaleX;
  newInfoMsg->K[1] = infoMsg->K[1] * scaleY;
  newInfoMsg->K[4] = infoMsg->K[4] * scaleY;
  newInfoMsg->K[2] = (infoMsg->K[2] - shiftX) * scaleX;
  newInfoMsg->K[5] = (infoMsg->K[5] - shiftY) * scaleY;
  newInfoMsg->K[8] = 1;
  newInfoMsg->P[0]  = newInfoMsg->K[0];
  newInfoMsg->P[1]  = newInfoMsg->K[1];
  newInfoMsg->P[2]  = newInfoMsg->K[2];
  newInfoMsg->P[4]  = newInfoMsg->K[3];
  newInfoMsg->P[5]  = newInfoMsg->K[4];
  newInfoMsg->P[6]  = newInfoMsg->K[5];
  newInfoMsg->P[8]  = newInfoMsg->K[6];
  newInfoMsg->P[9]  = newInfoMsg->K[7];
  newInfoMsg->P[10]  = newInfoMsg->K[8];
  newInfoMsg->P[3] = infoMsg->P[3] * scaleX;
  newInfoMsg->P[7] = infoMsg->P[7] * scaleY;
  newInfoMsg->P[11] = infoMsg->P[11];
  
  pubCamera_.publish(newImgMsg, newInfoMsg);
}

std::string chooseRightEncoding(const std::string& encoding) {
  namespace enc = sensor_msgs::image_encodings;
  if (encoding == enc::BAYER_RGGB8) return enc::RGB8;
  if (encoding == enc::BAYER_BGGR8) return enc::BGR8;
  if (encoding == enc::BAYER_RGGB16) return enc::RGB16;
  if (encoding == enc::BAYER_BGGR16) return enc::BGR16;
  else return encoding;
}

sensor_msgs::Image::ConstPtr Resize::handleImage (const sensor_msgs::Image::ConstPtr& imgMsg) {
  ROS_DEBUG_STREAM(name_ << " Resize Entered callback!");

  cv_bridge::CvImagePtr outImage = nullptr;
  try {
    outImage = cv_bridge::toCvCopy(imgMsg, chooseRightEncoding(imgMsg->encoding));

    // limit roi width
    const int roi_width = std::min(params_.roi_width, int(imgMsg->width) - params_.roi_x);
    const int roi_height = std::min(params_.roi_height, int(imgMsg->height) - params_.roi_y);
    if (params_.roi_x >= 0 && params_.roi_y >= 0 && roi_width > 0 && roi_height > 0) {
      outImage->image = outImage->image(cv::Rect(params_.roi_x, params_.roi_y, roi_width, roi_height));
    }

    if (params_.new_width >= 0 && params_.new_height >= 0) {
      cv::resize(outImage->image, outImage->image, cv::Size(params_.new_width, params_.new_height), 0, 0, CV_INTER_LINEAR);
    } else {
      cv::resize(outImage->image, outImage->image, cv::Size(0, 0), params_.scale, params_.scale, CV_INTER_LINEAR);
    }
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR_STREAM("Failed to convert ROS image into CV image: " << e.what());
  }

  ROS_DEBUG_STREAM(name_ << " Resize leaving callback!");
  if (outImage) {
    return outImage->toImageMsg();
  } else {
    return boost::make_shared<sensor_msgs::Image>();
  }
}
