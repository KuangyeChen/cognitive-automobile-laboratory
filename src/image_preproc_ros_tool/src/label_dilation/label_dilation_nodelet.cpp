#include <memory>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "label_dilation.hpp"

namespace image_preproc_ros_tool {

class LabelDilationNodelet : public nodelet::Nodelet {

    inline void onInit() override {
        impl_ = std::make_unique<LabelDilation>(getNodeHandle(), getPrivateNodeHandle());
    }
    std::unique_ptr<LabelDilation> impl_;
};
} // namespace image_preproc_ros_tool

PLUGINLIB_EXPORT_CLASS(image_preproc_ros_tool::LabelDilationNodelet, nodelet::Nodelet);
