#include <memory>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "pylon_detector.hpp"

namespace demo_image_processing_ros_tool {

class PylonDetectorNodelet : public nodelet::Nodelet {

    inline void onInit() override {
        impl_ = std::make_unique<PylonDetector>(getNodeHandle(), getPrivateNodeHandle());
    }
    std::unique_ptr<PylonDetector> impl_;
};
}

PLUGINLIB_EXPORT_CLASS(demo_image_processing_ros_tool::PylonDetectorNodelet, nodelet::Nodelet);
