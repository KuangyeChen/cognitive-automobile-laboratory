#include <memory>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "vehicle_simulator.hpp"

namespace simulation_ros_tool {

class VehicleSimulatorNodelet : public nodelet::Nodelet {

    inline void onInit() override {
        impl_ = std::make_unique<VehicleSimulator>(getNodeHandle(), getPrivateNodeHandle());
    }
    std::unique_ptr<VehicleSimulator> impl_;
};
} // namespace simulation_ros_tool

PLUGINLIB_EXPORT_CLASS(simulation_ros_tool::VehicleSimulatorNodelet, nodelet::Nodelet);
