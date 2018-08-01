#include <memory>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "kitaf_controller.hpp"

namespace kitaf_lateral_control_ros_tool {

class KitafControllerNodelet : public nodelet::Nodelet {

    inline void onInit() override {
        impl_ = std::make_unique<KitafController>(getNodeHandle(), getPrivateNodeHandle());
    }
    std::unique_ptr<KitafController> impl_;
};
} // namespace kitaf_lateral_control_ros_tool

PLUGINLIB_EXPORT_CLASS(kitaf_lateral_control_ros_tool::KitafControllerNodelet, nodelet::Nodelet);
