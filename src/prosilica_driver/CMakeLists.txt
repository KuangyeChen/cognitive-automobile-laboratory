cmake_minimum_required(VERSION 2.8.3)
project(prosilica_driver)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.

find_package(catkin REQUIRED COMPONENTS 
   prosilica_gige_sdk
   roscpp
   std_msgs
   sensor_msgs
   diagnostic_updater
   image_transport
   self_test 
   driver_base
   rosconsole
   dynamic_reconfigure 
   camera_calibration_parsers 
   polled_camera
   nodelet
   nodelet_topic_tools
   )

find_package(Boost REQUIRED COMPONENTS thread)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

# Generate dynamic parameters
generate_dynamic_reconfigure_options(cfg/ProsilicaCamera.cfg)
add_dependencies(${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

catkin_package()

add_library(prosilica src/libprosilica/prosilica.cpp)
target_link_libraries(prosilica ${catkin_LIBRARIES})

add_library(prosilica_nodelet src/nodes/prosilica_nodelet.cpp)
target_link_libraries(prosilica_nodelet prosilica ${catkin_LIBRARIES})
add_dependencies(prosilica_nodelet ${PROJECT_NAME}_gencfg)
class_loader_hide_library_symbols(prosilica_nodelet)

add_executable(prosilica_node src/nodes/prosilica_node.cpp)
target_link_libraries(prosilica_node 
   prosilica
   prosilica_nodelet
   ${Boost_LIBRARIES} 
   ${catkin_LIBRARIES})
add_dependencies(prosilica_node 
   ${catkin_EXPORTED_TARGETS}
   ${prosilica_driver_EXPORTED_TARGETS})

# disabled until it's mature??
#rosbuild_add_executable(find_camera find_camera.cpp)

add_executable(write_memory src/utilities/write_memory.cpp)
target_link_libraries(write_memory prosilica ${catkin_LIBRARIES})

add_executable(read_memory src/utilities/read_memory.cpp)
target_link_libraries(read_memory prosilica 
   ${catkin_LIBRARIES} 
   ${Boost_LIBRARIES})

add_executable(set_ip src/utilities/set_ip.cpp)
target_link_libraries(set_ip prosilica ${catkin_LIBRARIES})

add_executable(set_inhibition src/utilities/set_inhibition.cpp)
target_link_libraries(set_inhibition prosilica ${catkin_LIBRARIES})

install(TARGETS prosilica_node write_memory read_memory set_ip set_inhibition
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS prosilica prosilica_nodelet
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES prosilica.launch streaming.launch plugins/nodelet_plugins.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
