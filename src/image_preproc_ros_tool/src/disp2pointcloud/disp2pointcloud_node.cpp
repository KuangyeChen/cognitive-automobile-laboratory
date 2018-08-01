#include "disp2pointcloud.hpp"

int main(int argc, char *argv[]) {

  ros::init(argc, argv, "disp2pointcloud_node");

  image_preproc_ros_tool::Disp2pointcloud disp2pointcloud(ros::NodeHandle(),
                                                          ros::NodeHandle("~"));

  ros::spin();
  return EXIT_SUCCESS;
}
