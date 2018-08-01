add_executable(reprojection_visualizer 
    ReprojectionVisualizer.cpp
    ReprojectionVisualizer.h
    ReprojectionVisualizerParameters.cpp
    ReprojectionVisualizerParameters.h
    reprojection_visualizer_node.cpp
    )

add_dependencies(reprojection_visualizer 
    ${catkin_EXPORTED_TARGETS}
    )

target_link_libraries(reprojection_visualizer
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    yaml-cpp
    )
