add_executable(landmark_visualizer 
    LandmarkVisualizerParameters.cpp    
    LandmarkVisualizerParameters.h
    landmark_visualizer.cpp
    )

add_dependencies(landmark_visualizer 
    ${catkin_EXPORTED_TARGETS}
    )

target_link_libraries(landmark_visualizer
    ${catkin_LIBRARIES}
    yaml-cpp
    )
