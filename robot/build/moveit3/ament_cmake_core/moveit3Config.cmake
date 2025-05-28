# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_moveit3_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED moveit3_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(moveit3_FOUND FALSE)
  elseif(NOT moveit3_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(moveit3_FOUND FALSE)
  endif()
  return()
endif()
set(_moveit3_CONFIG_INCLUDED TRUE)

# output package information
if(NOT moveit3_FIND_QUIETLY)
  message(STATUS "Found moveit3: 0.3.0 (${moveit3_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'moveit3' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${moveit3_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(moveit3_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${moveit3_DIR}/${_extra}")
endforeach()
