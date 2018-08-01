cmake_minimum_required(VERSION 2.8.12)
project(stargazer_ros_tool)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
if(COMPILER_SUPPORTS_CXX14)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support. Please use a different C++ compiler.")
endif()

###################
## find packages ##
###################
find_package(catkin REQUIRED COMPONENTS roscpp cv_bridge image_transport sensor_msgs std_msgs
        cmake_modules dynamic_reconfigure message_generation stargazer rosbag nodelet tf2 tf2_geometry_msgs)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
    DIRECTORY msg
    FILES Landmark.msg LandmarkArray.msg LandmarkPoint.msg
    )
generate_messages(DEPENDENCIES std_msgs)

# Generate dynamic reconfigure options
generate_dynamic_reconfigure_options(
    cfg/LandmarkFinderInterface.cfg
    cfg/LandmarkLocalizerInterface.cfg
    )

###################################
## catkin specific configuration ##
###################################
catkin_package(
    DEPENDS message_runtime std_msgs
    )

###########
## Build ##
###########
# Add include and library directories
include_directories(
    ${Eigen_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    )

add_subdirectory(src/intrinsics_tool)
add_subdirectory(src/landmark_calibrator)
add_subdirectory(src/landmark_finder)
add_subdirectory(src/landmark_localizer)
add_subdirectory(src/landmark_visualizer)
add_subdirectory(src/pose_visualizer)
add_subdirectory(src/reprojection_visualizer)

#############
## Install ##
#############
# Install all targets, headers by default and scripts and other files if specified (folders or files)
#mrt_install(PROGRAMS scripts FILES launch rviz maps res data nodelet_plugins.xml plugin_description.xml)
