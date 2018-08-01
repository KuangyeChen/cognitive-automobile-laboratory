#include <memory>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "path_planning.hpp"

namespace kitaf_navigation_ros_tool {

class PathPlanningNodelet : public nodelet::Nodelet {

    inline void onInit() override {
        impl_ = std::make_unique<PathPlanning>(getNodeHandle(), getPrivateNodeHandle());
    }
    std::unique_ptr<PathPlanning> impl_;
};
} // namespace kitaf_navigation_ros_tool

PLUGINLIB_EXPORT_CLASS(kitaf_navigation_ros_tool::PathPlanningNodelet, nodelet::Nodelet);
