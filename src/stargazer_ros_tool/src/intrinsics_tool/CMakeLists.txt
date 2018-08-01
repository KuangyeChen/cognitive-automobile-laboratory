add_executable(intrinsics_tool
   intrinsics_tool.cpp
   )

add_dependencies(intrinsics_tool
   ${catkin_EXPORTED_TARGETS}
   )

target_link_libraries(intrinsics_tool 
    ${catkin_LIBRARIES}
    yaml-cpp
    )
