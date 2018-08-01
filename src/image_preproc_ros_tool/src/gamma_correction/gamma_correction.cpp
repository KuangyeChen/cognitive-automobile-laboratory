#include "gamma_correction.h"

GammaCorrector::GammaCorrector(ros::NodeHandle& nodeHandle, ros::NodeHandle& privateNodeHandle, const std::string name) {
  name_ = name;
  double fGamma = 0.0;
  privateNodeHandle.param("gamma", fGamma, 1.0);
  privateNodeHandle.param("auto_gamma", autoGamma_, false);

  std::string srcTopic = nodeHandle.resolveName("src/image");
  std::string tgtTopic = nodeHandle.resolveName("tgt/image");

  srv_.reset(new ReconfigureServer(privateNodeHandle));
  dynamic_reconfigure::Server<image_preproc_ros_tool::GammaCorrectionConfig>::CallbackType f(
          boost::bind(&GammaCorrector::reconfigureRequest, this, _1, _2));
  srv_->setCallback(f);

  pub_ = nodeHandle.advertise<sensor_msgs::Image>(tgtTopic, 10);
  sub_ = nodeHandle.subscribe<sensor_msgs::Image>(srcTopic, 10,
                                                  &GammaCorrector::handleImage, this,
                                                  ros::TransportHints().tcpNoDelay());
}

void GammaCorrector::reconfigureRequest(const image_preproc_ros_tool::GammaCorrectionConfig& cfg, uint32_t) {
  gamma_ = cfg.gamma;
  autoGamma_ = cfg.auto_gamma;
}

void GammaCorrector::handleImage (const sensor_msgs::Image::ConstPtr& imgMsg) {
  ROS_INFO_STREAM(name_ << " Gamma correction Entered callback!");
    try {
        cv_bridge::CvImagePtr cvPtr = cv_bridge::toCvCopy(imgMsg);
        cv_bridge::CvImage outImage;
        outImage.header = imgMsg->header;
        outImage.encoding = imgMsg->encoding;

        if(autoGamma_) {
          cv::Mat img;
          if(cvPtr->image.channels() == 3) {
            cv::cvtColor(cvPtr->image, img, CV_BGR2GRAY);
          } else if(cvPtr->image.channels() == 1) {
            img = cvPtr->image;
          }
          double gamma = image_preproc::BrightnessCorrection::computeOptimalGamma(img);
          image_preproc::BrightnessCorrection::correctGamma(cvPtr->image, outImage.image, gamma*gamma_);

        } else {
          image_preproc::BrightnessCorrection::correctGamma(cvPtr->image, outImage.image, gamma_);
        }

        pub_.publish(outImage);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR_STREAM("Failed to convert ROS image into CV image: " << e.what());
    }
  ROS_INFO_STREAM(name_ << " gamma correction leaving callback!");

}

