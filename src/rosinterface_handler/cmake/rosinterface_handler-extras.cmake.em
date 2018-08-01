# Generated from: rosinterface_handler/cmake/rosinterface_handler-extras.cmake.em

if (_ROSINTERFACE_HANDLER_EXTRAS_INCLUDED_)
    return()
endif ()
set(_ROSINTERFACE_HANDLER_EXTRAS_INCLUDED_ TRUE)

@[if DEVELSPACE]@
# cmake dir in develspace
set(ROSINTERFACE_HANDLER_CMAKE_DIR "@(CMAKE_CURRENT_SOURCE_DIR)/cmake")
@[else]@
# cmake dir in installspace
set(ROSINTERFACE_HANDLER_CMAKE_DIR "@(PKG_CMAKE_DIR)")
@[end if]@

include(${ROSINTERFACE_HANDLER_CMAKE_DIR}/rosinterface_handler-macros.cmake)
