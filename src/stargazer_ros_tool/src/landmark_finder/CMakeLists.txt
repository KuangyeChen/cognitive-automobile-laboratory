set(LANDMARK_FINDER_SRCS
    LandmarkFinderInterface.cpp
    LandmarkFinderInterface.h
    LandmarkFinderInterfaceParameters.cpp
    LandmarkFinderInterfaceParameters.h
   )

set(LANDMARK_FINDER_DEPS
    ${catkin_EXPORTED_TARGETS}
    )

set(LANDMARK_FINDER_LIBS
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    )

add_executable(landmark_finder 
    ${LANDMARK_FINDER_SRCS}
    landmark_finder_node.cpp
    )
add_dependencies(landmark_finder ${LANDMARK_FINDER_DEPS})
target_link_libraries(landmark_finder ${LANDMARK_FINDER_LIBS})

add_library(landmark_finder_nodelet
    ${LANDMARK_FINDER_SRCS}
    landmark_finder_nodelet.cpp
    )

add_dependencies(landmark_finder_nodelet ${LANDMARK_FINDER_DEPS})
target_link_libraries(landmark_finder_nodelet ${LANDMARK_FINDER_LIBS})
