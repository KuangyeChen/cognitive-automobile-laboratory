#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "LandmarkFinderInterface.h"

namespace stargazer_ros_tool {

class LandmarkFinderInterfaceNodelet : public nodelet::Nodelet {

    inline void onInit() override {
        m_ = std::make_unique<LandmarkFinderInterface>(getNodeHandle(), getPrivateNodeHandle());
    }
    std::unique_ptr<LandmarkFinderInterface> m_;
};
}

PLUGINLIB_EXPORT_CLASS(stargazer_ros_tool::LandmarkFinderInterfaceNodelet, nodelet::Nodelet);
