cmake_minimum_required(VERSION 3.5)
project(costmap_converter)
 
# Set to Release to speed up the build
set(CMAKE_BUILD_TYPE Release)
 
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
 
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
 
find_package(ament_cmake REQUIRED)
find_package(class_loader REQUIRED)
find_package(costmap_converter_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
 
set(dependencies
  class_loader
  costmap_converter_msgs
  cv_bridge
  geometry_msgs
  nav2_costmap_2d
  tf2
  tf2_geometry_msgs
  tf2_ros
  OpenCV
  pluginlib
  rclcpp
  rclcpp_components
)
 
include_directories(
  include
#  /opt/ros/jazzy/include/cv_bridge/cv_bridge
)
 
# Core costmap_converter library
add_library(costmap_converter SHARED
  src/costmap_to_polygons.cpp 
  src/costmap_to_polygons_concave.cpp
  src/costmap_to_lines_convex_hull.cpp
  src/costmap_to_lines_ransac.cpp
  src/costmap_to_dynamic_obstacles/costmap_to_dynamic_obstacles.cpp
  src/costmap_to_dynamic_obstacles/background_subtractor.cpp
  src/costmap_to_dynamic_obstacles/blob_detector.cpp
  src/costmap_to_dynamic_obstacles/multitarget_tracker/Ctracker.cpp
  src/costmap_to_dynamic_obstacles/multitarget_tracker/Kalman.cpp
  src/costmap_to_dynamic_obstacles/multitarget_tracker/HungarianAlg.cpp
)
 
ament_target_dependencies(costmap_converter ${dependencies})
 
# Register costmap_converter_node as a component (NOT an executable)
add_library(costmap_converter_node SHARED
  src/costmap_converter_node.cpp
)
 
ament_target_dependencies(costmap_converter_node ${dependencies})
 
target_link_libraries(costmap_converter_node costmap_converter)
 
rclcpp_components_register_nodes(costmap_converter_node
  "CostmapStandaloneConversion"
)
 
# Installation
install(TARGETS
  costmap_converter
  costmap_converter_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
 
install(DIRECTORY include/
  DESTINATION include/
)
 
ament_export_include_directories(include)
ament_export_libraries(costmap_converter)
ament_export_dependencies(${dependencies})
 
pluginlib_export_plugin_description_file(costmap_converter costmap_converter_plugins.xml)
 
# Optional: Unit test
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_costmap_polygons test/costmap_polygons.cpp)
  if(TARGET test_costmap_polygons)
    target_link_libraries(test_costmap_polygons costmap_converter)
  endif()
  ament_target_dependencies(test_costmap_polygons ${dependencies})
endif()
 
ament_package()
 
