
macro(generate_ros_interface_files)
    set(CFG_FILES "${ARGN}")
    set(ROSINTERFACE_HANDLER_ROOT_DIR "${ROSINTERFACE_HANDLER_CMAKE_DIR}/..")
    if (${PROJECT_NAME}_CATKIN_PACKAGE)
        message(FATAL_ERROR "generate_interface_files() must be called before catkin_package() in project '${PROJECT_NAME}'")
    endif ()

    # ensure that package destination variables are defined
    catkin_destinations()
    if(dynamic_reconfigure_FOUND_CATKIN_PROJECT)
        add_definitions(-DDYNAMIC_RECONFIGURE_FOUND)
    endif()
    if(message_filters_FOUND_CATKIN_PROJECT)
        add_definitions(-DMESSAGE_FILTERS_FOUND)
    endif()

    set(_autogen "")
    foreach (_cfg ${CFG_FILES})
        # Construct the path to the .cfg file
        set(_input ${_cfg})
        if (NOT IS_ABSOLUTE ${_input})
            set(_input ${PROJECT_SOURCE_DIR}/${_input})
        endif ()

        get_filename_component(_cfgext ${_cfg} EXT)
        if( _cfgext STREQUAL ".rosif")
            # Define required input files
            set(geninterface_build_files
                    ${ROSINTERFACE_HANDLER_ROOT_DIR}/templates/ConfigType.h.template
                    ${ROSINTERFACE_HANDLER_ROOT_DIR}/templates/Interface.h.template
                    )

            # Define output files
            get_filename_component(_cfgonly ${_cfg} NAME_WE)
            set(_output_cfg ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/cfg/${_cfgonly}.cfg)
            set(_output_cpp ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}/${_cfgonly}Interface.h)
            set(_output_py ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/interface/${_cfgonly}Interface.py)

            # We need to explicitly add the devel space to the PYTHONPATH
            # since it might contain dynamic_reconfigure or Python code of the current package.
            set("_CUSTOM_PYTHONPATH_ENV")
            if(EXISTS "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION}")
                configure_file(
                    "${ROSINTERFACE_HANDLER_CMAKE_DIR}/setup_custom_pythonpath_rosif.sh.in"
                    "setup_custom_pythonpath_rosif.sh"
                    @ONLY
                )
                set("_CUSTOM_PYTHONPATH_ENV" "${CMAKE_CURRENT_BINARY_DIR}/setup_custom_pythonpath_rosif.sh")
            endif()

            # Create command
            assert(CATKIN_ENV)
            set(_cmd
                    ${CATKIN_ENV}
                    ${_CUSTOM_PYTHONPATH_ENV}
                    ${_input}
                    ${ROSINTERFACE_HANDLER_ROOT_DIR}
                    ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}
                    ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}
                    ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
                    )

            add_custom_command(OUTPUT
                    ${_output_cpp} ${_output_cfg} ${_output_py}
                    COMMAND ${_cmd}
                    DEPENDS ${_input} ${geninterface_build_files}
                    COMMENT "Generating interface files from ${_cfgonly}"
                    )

            list(APPEND ${PROJECT_NAME}_LOCAL_CFG_FILES "${_output_cfg}")
            list(APPEND ${PROJECT_NAME}_interfaces_generated ${_output_cpp} ${_output_cfg} ${_output_py})

            # make file show up in ides
            STRING(REGEX REPLACE "/" "-" IDE_TARGET_NAME ${PROJECT_NAME}-show-cfg-${_cfgonly})
            add_custom_target(${IDE_TARGET_NAME} SOURCES ${_input})

            install(
                    FILES ${_output_cpp}
                    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
            )
        elseif( _cfgext STREQUAL ".cfg" )
            list(APPEND ${PROJECT_NAME}_LOCAL_CFG_FILES "${_cfg}")
        elseif( _cfgext STREQUAL ".param" )
            message(WARNING "Found old .param Files. Rosparam handler and rosinterface handler should not be used in combination!")
        else()
            message(WARNING "Unknown file ending : ${_cfgext}. Skipping")
        endif()

    endforeach (_cfg)

    # geninterface target for hard dependency on generate_interface generation
    add_custom_target(${PROJECT_NAME}_geninterface ALL DEPENDS ${${PROJECT_NAME}_interfaces_generated})

    # register target for catkin_package(EXPORTED_TARGETS)
    list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ${PROJECT_NAME}_geninterface)

    # make sure we can find generated headers and that they overlay all other includes
    include_directories(BEFORE ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION})
    # pass the include directory to catkin_package()
    list(APPEND ${PROJECT_NAME}_INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION})
    # ensure that the folder exists
    file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION})
    #Require C++11
    set_property(TARGET ${PROJECT_NAME}_geninterface PROPERTY CXX_STANDARD 11)
    set_property(TARGET ${PROJECT_NAME}_geninterface PROPERTY CXX_STANDARD_REQUIRED ON)

    # install python files
    install_ros_python_interface_files()

    # generate dynamic reconfigure files
    if(dynamic_reconfigure_FOUND_CATKIN_PROJECT)
        if(${PROJECT_NAME}_LOCAL_CFG_FILES)
            generate_dynamic_reconfigure_options(${${PROJECT_NAME}_LOCAL_CFG_FILES})
        endif()
    else()
        message(WARNING "Dependency to dynamic_reconfigure is missing, or find_package(dynamic_reconfigure) was not called yet. Not building dynamic config files")
    endif()
endmacro()

macro(install_ros_python_interface_files)
    if(NOT install_ros_python_interface_files_CALLED)
        set(install_ros_python_interface_files_CALLED TRUE)

        # mark that generate_dynamic_reconfigure_options() was called in order to detect wrong order of calling with catkin_python_setup()
        set(${PROJECT_NAME}_GENERATE_DYNAMIC_RECONFIGURE TRUE)

        # check if catkin_python_setup() installs an __init__.py file for a package with the current project name
        # in order to skip the installation of a generated __init__.py file
        set(package_has_static_sources ${${PROJECT_NAME}_CATKIN_PYTHON_SETUP_HAS_PACKAGE_INIT})
        if(NOT EXISTS ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/__init__.py)
          file(WRITE ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/__init__.py "")
        endif()
        if(NOT package_has_static_sources)
            # install package __init__.py
            install(
               FILES ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/__init__.py
               DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
               )
        endif()
            
        # generate interface module __init__.py
        if(NOT EXISTS ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/interface/__init__.py)
            file(WRITE ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/interface/__init__.py "")
        endif()
        
        # compile python code before installing
        find_package(PythonInterp REQUIRED)
        install(CODE "execute_process(COMMAND \"${PYTHON_EXECUTABLE}\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/interface\")")
        install(
            DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/interface
            DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
            )
    endif()
endmacro()
