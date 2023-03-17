cmake_minimum_required(VERSION 3.8)
project(spice_nav)

set(lib_name ${PROJECT_NAME}_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(spice_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(
  include
)

set(dep_pkgs
  rclcpp
  nav2_costmap_2d
  pluginlib
  geometry_msgs
  tf2_ros
  tf2_msgs
  tf2
  )

add_library(${lib_name} SHARED
  src/DynamicObstacleLayer.cpp)
include_directories(include)

install(TARGETS ${lib_name}
  DESTINATION lib)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(PROGRAMS
  scripts/dynamic_obstacle_avoidance.py
  DESTINATION lib/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(nav2_costmap_2d dynamic_obstacle_layer.xml)
ament_target_dependencies(${lib_name} ${dep_pkgs})

ament_package()
