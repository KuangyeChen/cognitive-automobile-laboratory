#include <ros/ros.h>
#include <opencv/cv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>
#include <utils_ros/smart_subscriber.hpp>

#include "image_preproc_ros_tool/ResizeInterface.h"

class Resize {
  using Parameters = image_preproc_ros_tool::ResizeInterface;
  using Config = image_preproc_ros_tool::ResizeConfig;
  using ReconfigureServer = dynamic_reconfigure::Server<Config>;
public:
  Resize(ros::NodeHandle& nodeHandle, ros::NodeHandle& privateNodeHandle, const std::string name);

private:

  sensor_msgs::Image::ConstPtr handleImage(const sensor_msgs::Image::ConstPtr& imgMsg);
  void imageCallback(const sensor_msgs::Image::ConstPtr& imgMsg);
  void cameraCallback(const sensor_msgs::ImageConstPtr& imgMsg,
                      const sensor_msgs::CameraInfoConstPtr& infoMsg);
  void reconfigureRequest(const Config &, uint32_t);
  void connectCallback();

  Parameters params_;
  ReconfigureServer reconfigureServer_;

  ros::Publisher pub_;
  utils_ros::SmartSubscriber<sensor_msgs::Image> sub_;
  std::unique_ptr<image_transport::ImageTransport> imageTransport_;
  image_transport::CameraPublisher pubCamera_;
  image_transport::CameraSubscriber subCamera_;
  std::string name_;
};
