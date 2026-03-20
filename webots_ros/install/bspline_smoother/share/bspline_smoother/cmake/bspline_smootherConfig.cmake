# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_bspline_smoother_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED bspline_smoother_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(bspline_smoother_FOUND FALSE)
  elseif(NOT bspline_smoother_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(bspline_smoother_FOUND FALSE)
  endif()
  return()
endif()
set(_bspline_smoother_CONFIG_INCLUDED TRUE)

# output package information
if(NOT bspline_smoother_FIND_QUIETLY)
  message(STATUS "Found bspline_smoother: 0.0.0 (${bspline_smoother_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'bspline_smoother' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT bspline_smoother_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(bspline_smoother_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_include_directories-extras.cmake;ament_cmake_export_libraries-extras.cmake;ament_cmake_export_dependencies-extras.cmake")
foreach(_extra ${_extras})
  include("${bspline_smoother_DIR}/${_extra}")
endforeach()
