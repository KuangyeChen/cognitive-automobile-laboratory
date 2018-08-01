#include "resize.h"

int main (int argc, char *argv[]) {
    // ROS setup and parameter handling
    ros::init(argc, argv, "resize", ros::init_options::AnonymousName);

    ros::NodeHandle publicNH;
    ros::NodeHandle privateNH("~");


    Resize resizer(publicNH, privateNH, ros::this_node::getName());
    ros::spin();
    exit(EXIT_SUCCESS);
}
