#include <memory>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "map_creation.hpp"

namespace map_creation_ros_tool {

class MapCreationNodelet : public nodelet::Nodelet {

    inline void onInit() override {
        impl_ = std::make_unique<MapCreation>(getNodeHandle(), getPrivateNodeHandle());
    }
    std::unique_ptr<MapCreation> impl_;
};
} // namespace map_creation_ros_tool

PLUGINLIB_EXPORT_CLASS(map_creation_ros_tool::MapCreationNodelet, nodelet::Nodelet);
