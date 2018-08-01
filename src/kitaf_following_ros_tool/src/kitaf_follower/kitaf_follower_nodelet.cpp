#include <memory>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "kitaf_follower.hpp"

namespace kitaf_following_ros_tool {

class KitafFollowerNodelet : public nodelet::Nodelet {

    inline void onInit() override {
        impl_ = std::make_unique<KitafFollower>(getNodeHandle(), getPrivateNodeHandle());
    }
    std::unique_ptr<KitafFollower> impl_;
};
} // namespace kitaf_following_ros_tool

PLUGINLIB_EXPORT_CLASS(kitaf_following_ros_tool::KitafFollowerNodelet, nodelet::Nodelet);
