#include <memory>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "gamma_correction.h"

namespace image_preproc_ros_tool {

class GammaCorrectionNodelet : public nodelet::Nodelet {

    virtual void onInit();
    std::unique_ptr<GammaCorrector> m_;
};

void GammaCorrectionNodelet::onInit() {
    m_ = std::make_unique<GammaCorrector>(this->getNodeHandle(), this->getPrivateNodeHandle(), this->getName());
}
}

PLUGINLIB_DECLARE_CLASS(image_preproc_ros_tool, GammaCorrectionNodelet,
                        image_preproc_ros_tool::GammaCorrectionNodelet, nodelet::Nodelet);
