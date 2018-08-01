set(MRT_PKG_VERSION 2.2.1)
# Modify only if you know what you are doing!
cmake_minimum_required(VERSION 2.8.12)
project(rosparam_handler)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package(
        CATKIN_DEPENDS catkin
        CFG_EXTRAS rosparam_handler-extras.cmake rosparam_handler-macros.cmake
)

install(
        DIRECTORY templates
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
