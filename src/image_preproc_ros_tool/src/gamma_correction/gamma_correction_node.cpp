#include "gamma_correction.h"

int main (int argc, char *argv[]) {
    // ROS setup and parameter handling
    ros::init(argc, argv, "gamma_correction", ros::init_options::AnonymousName);

    ros::NodeHandle publicNH;
    ros::NodeHandle privateNH("~");


    GammaCorrector corrector(publicNH, privateNH, ros::this_node::getName());
    ros::spin();
    exit(EXIT_SUCCESS);
}
