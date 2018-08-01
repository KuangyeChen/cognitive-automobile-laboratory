add_executable(pose_visualizer 
    PoseVisualizerParameters.cpp
    PoseVisualizerParameters.h
    pose_visualizer.cpp
    )

add_dependencies(pose_visualizer 
    ${catkin_EXPORTED_TARGETS}
    )

target_link_libraries(pose_visualizer
    ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
    )
