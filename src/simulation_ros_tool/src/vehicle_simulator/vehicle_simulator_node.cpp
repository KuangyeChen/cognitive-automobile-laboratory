#include "vehicle_simulator.hpp"

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "vehicle_simulator_node");

    simulation_ros_tool::VehicleSimulator vehicle_simulator(ros::NodeHandle(), ros::NodeHandle("~"));

    ros::spin();
    return EXIT_SUCCESS;
}
