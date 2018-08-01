#include <memory>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "filter_and_control_tar.hpp"

namespace kitaf_navigation_ros_tool {

class FilterAndControlTarNodelet : public nodelet::Nodelet {

    inline void onInit() override {
        impl_ = std::make_unique<FilterAndControlTar>(getNodeHandle(), getPrivateNodeHandle());
    }
    std::unique_ptr<FilterAndControlTar> impl_;
};
} // namespace kitaf_navigation_ros_tool

PLUGINLIB_EXPORT_CLASS(kitaf_navigation_ros_tool::FilterAndControlTarNodelet, nodelet::Nodelet);
