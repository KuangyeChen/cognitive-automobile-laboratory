cmake_minimum_required(VERSION 2.8.3)
project(rosinterface_handler)

find_package(catkin REQUIRED)

catkin_python_setup()

if (CATKIN_ENABLE_TESTING)
	# set compiler flags
    find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure rostest roscpp message_filters std_msgs)
    set(ROSINTERFACE_HANDLER_CMAKE_DIR ${CMAKE_CURRENT_LIST_DIR}/cmake)
    include(cmake/rosinterface_handler-macros.cmake)
        file(GLOB PROJECT_TEST_FILES_INTERFACE RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "test/cfg/*.rosif")
        generate_ros_interface_files(${PROJECT_TEST_FILES_INTERFACE})
endif()

# mark template files for IDEs
file(GLOB TEMPLATE_FILES RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "templates/*")
add_custom_target(rosinterface_handler_templates SOURCES ${TEMPLATE_FILES})
file(GLOB PYTEST_FILES RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "test/python/tests/*")
add_custom_target(rosinterface_handler_pytests SOURCES ${PYTEST_FILES})
file(GLOB PY_FILES RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "src/rosinterface_handler/*")
add_custom_target(rosinterface_handler_py SOURCES ${PY_FILES})

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS catkin
        CFG_EXTRAS rosinterface_handler-extras.cmake
)

install(
        DIRECTORY cmake
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
        DIRECTORY templates
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
        DIRECTORY include/rosinterface_handler/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(PROGRAMS scripts/generate_yaml
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
    include(CheckCXXCompilerFlag)
    CHECK_CXX_COMPILER_FLAG("-std=c++14" Cpp14CompilerFlag)
    if (${Cpp14CompilerFlag})
           set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
    else()
           message(FATAL_ERROR "Compiler does not have c++14 support.")
    endif()

    file(GLOB PROJECT_TEST_FILES_SRC RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "test/src/*.cpp")
    set(TEST_TARGET_NAME "rosinterface_handler_test")
    add_rostest_gtest(${TEST_TARGET_NAME} test/launch/rosinterface_handler.test ${PROJECT_TEST_FILES_SRC})
    add_rostest(test/launch/rosinterface_handler_python.test DEPENDENCIES ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
    find_package(tf2_ros REQUIRED)
    target_link_libraries(${TEST_TARGET_NAME} ${catkin_LIBRARIES} ${tf2_ros_LIBRARIES} gtest)
    target_include_directories(${TEST_TARGET_NAME} PUBLIC ${catkin_INCLUDE_DIRS} ${tf2_ros_INCLUDE_DIRS} include)
    add_dependencies(${TEST_TARGET_NAME} ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
    set_property(TARGET ${TEST_TARGET_NAME} PROPERTY CXX_STANDARD 14)
    set_property(TARGET ${TEST_TARGET_NAME} PROPERTY CXX_STANDARD_REQUIRED ON)
endif()
