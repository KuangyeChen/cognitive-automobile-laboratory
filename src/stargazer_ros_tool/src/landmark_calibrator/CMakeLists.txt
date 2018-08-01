add_executable(landmark_calibrator 
    LandmarkCalibratorInterface.cpp
    LandmarkCalibratorInterface.h
    LandmarkCalibratorInterfaceParameters.cpp
    LandmarkCalibratorInterfaceParameters.h
    landmark_calibrator_node.cpp
    )

add_dependencies(landmark_calibrator 
    ${catkin_EXPORTED_TARGETS}
    )

target_link_libraries(landmark_calibrator
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${CERES_LIBRARIES}
    yaml-cpp
    )
