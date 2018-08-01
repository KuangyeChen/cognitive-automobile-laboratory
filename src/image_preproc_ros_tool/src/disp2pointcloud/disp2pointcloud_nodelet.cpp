#include <memory>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "disp2pointcloud.hpp"

namespace image_preproc_ros_tool {

class Disp2pointcloudNodelet : public nodelet::Nodelet {

  inline void onInit() override {
    impl_ = std::make_unique<Disp2pointcloud>(getNodeHandle(),
                                              getPrivateNodeHandle());
  }
  std::unique_ptr<Disp2pointcloud> impl_;
};
} // namespace image_preproc_ros_tool

PLUGINLIB_EXPORT_CLASS(image_preproc_ros_tool::Disp2pointcloudNodelet,
                       nodelet::Nodelet);
