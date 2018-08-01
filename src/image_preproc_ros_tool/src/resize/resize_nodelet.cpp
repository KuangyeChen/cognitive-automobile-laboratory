#include <memory>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "resize.h"

namespace image_preproc_ros_tool {

class ResizeNodelet : public nodelet::Nodelet {

    virtual void onInit();
    std::unique_ptr<Resize> m_;
};

void ResizeNodelet::onInit() {
    m_ = std::make_unique<Resize>(this->getNodeHandle(), this->getPrivateNodeHandle(), this->getName());
}
}

PLUGINLIB_DECLARE_CLASS(image_preproc_ros_tool, ResizeNodelet,
                        image_preproc_ros_tool::ResizeNodelet, nodelet::Nodelet);
