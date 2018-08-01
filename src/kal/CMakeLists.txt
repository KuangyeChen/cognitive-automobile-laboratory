#pkg_version=1.4
cmake_minimum_required(VERSION 2.8.3)
project(kal)

###################
## find packages ##
###################
find_package(mrt_cmake_modules REQUIRED)
include(GatherDeps)

###################################
## catkin specific configuration ##
###################################
# The catkin_package macro generates cmake config files for your package
# Declare things to be passed to dependent projects
# INCLUDE_DIRS: uncomment this if you package contains header files
# LIBRARIES: libraries you create in this project that dependent projects also need
# CATKIN_DEPENDS: catkin_packages dependent projects also need
# DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS ${mrt_EXPORT_INCLUDE_DIRS}
  LIBRARIES  ${mrt_EXPORT_LIBRARIES}
  CATKIN_DEPENDS ${catkin_EXPORT_DEPENDS}
)

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch)
  roslaunch_add_file_check(launch)
endif()
