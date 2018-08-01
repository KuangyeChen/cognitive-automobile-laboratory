#include <memory>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "path_provider.hpp"

namespace path_provider_ros_tool {

class PathProviderNodelet : public nodelet::Nodelet {

    inline void onInit() override {
        impl_ = std::make_unique<PathProvider>(getNodeHandle(), getPrivateNodeHandle());
    }
    std::unique_ptr<PathProvider> impl_;
};
}

PLUGINLIB_EXPORT_CLASS(path_provider_ros_tool::PathProviderNodelet, nodelet::Nodelet);
