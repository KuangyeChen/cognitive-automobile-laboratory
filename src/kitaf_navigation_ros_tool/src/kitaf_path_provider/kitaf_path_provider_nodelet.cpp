#include <memory>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "kitaf_path_provider.hpp"

namespace kitaf_navigation_ros_tool {

class KitafPathProviderNodelet : public nodelet::Nodelet {

    inline void onInit() override {
        impl_ = std::make_unique<KitafPathProvider>(getNodeHandle(), getPrivateNodeHandle());
    }
    std::unique_ptr<KitafPathProvider> impl_;
};
} // namespace kitaf_navigation_ros_tool

PLUGINLIB_EXPORT_CLASS(kitaf_navigation_ros_tool::KitafPathProviderNodelet, nodelet::Nodelet);
