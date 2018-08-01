set(LANDMARK_LOCALIZER_SRCS
    LandmarkLocalizerInterface.cpp
    LandmarkLocalizerInterface.h
    LandmarkLocalizerInterfaceParameters.cpp
    LandmarkLocalizerInterfaceParameters.h
   )

set(LANDMARK_LOCALIZER_DEPS
    ${catkin_EXPORTED_TARGETS}
    )

set(LANDMARK_LOCALIZER_LIBS
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${CERES_LIBRARIES}
    )

add_executable(landmark_localizer 
    ${LANDMARK_LOCALIZER_SRCS}
    landmark_localizer_node.cpp
    )
add_dependencies(landmark_localizer ${LANDMARK_LOCALIZER_DEPS})
target_link_libraries(landmark_localizer ${LANDMARK_LOCALIZER_LIBS})

add_library(landmark_localizer_nodelet
    ${LANDMARK_LOCALIZER_SRCS}
    landmark_localizer_nodelet.cpp
    )

add_dependencies(landmark_localizer_nodelet ${LANDMARK_LOCALIZER_DEPS})
target_link_libraries(landmark_localizer_nodelet ${LANDMARK_LOCALIZER_LIBS})
