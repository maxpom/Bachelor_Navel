cmake_minimum_required(VERSION 3.8)
project(bspline_smoother)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
set(dependencies
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav2_util
  nav2_costmap_2d
  pluginlib
  geometry_msgs
  nav_msgs
)

add_library(${PROJECT_NAME} SHARED
  src/bspline_smoother.cpp
  src/bspline.cpp
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}  
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen                      
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

pluginlib_export_plugin_description_file(nav2_core smoother_plugin.xml)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
