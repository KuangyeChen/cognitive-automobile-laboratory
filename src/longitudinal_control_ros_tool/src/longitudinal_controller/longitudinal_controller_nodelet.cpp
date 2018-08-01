#include <memory>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "longitudinal_controller.hpp"

namespace longitudinal_control_ros_tool {

class LongitudinalControllerNodelet : public nodelet::Nodelet {

    inline void onInit() override {
        impl_ = std::make_unique<LongitudinalController>(getNodeHandle(), getPrivateNodeHandle());
    }
    std::unique_ptr<LongitudinalController> impl_;
};
} // namespace longitudinal_control_ros_tool

PLUGINLIB_EXPORT_CLASS(longitudinal_control_ros_tool::LongitudinalControllerNodelet, nodelet::Nodelet);
