#include <ros/ros.h>
#include <opencv/cv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <dynamic_reconfigure/server.h>
#include <image_preproc/brightness_correction.h>
#include <image_preproc_ros_tool/GammaCorrectionConfig.h>


class GammaCorrector {
  using ReconfigureServer = dynamic_reconfigure::Server<image_preproc_ros_tool::GammaCorrectionConfig>;
public:
  GammaCorrector(ros::NodeHandle& nodeHandle, ros::NodeHandle& privateNodeHandle, const std::string name);

private:

  void reconfigureRequest(const image_preproc_ros_tool::GammaCorrectionConfig& cfg, uint32_t);
  void handleImage (const sensor_msgs::Image::ConstPtr& imgMsg);


  ros::Publisher pub_;
  ros::Subscriber sub_;
  std::unique_ptr<ReconfigureServer> srv_;
  double gamma_;
  bool autoGamma_;
  std::string name_;
};
