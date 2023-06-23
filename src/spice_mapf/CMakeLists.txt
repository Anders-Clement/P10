cmake_minimum_required(VERSION 3.8)
project(spice_mapf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(spice_msgs)
find_package(spice_mapf_msgs)
find_package(tf2)
find_package(tf2_geometry_msgs)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

include_directories(
  include
)

add_executable(mapf_navigator_node_cpp src/mapf_navigator_node.cpp)
ament_target_dependencies(
    mapf_navigator_node_cpp
    geometry_msgs
    rclcpp
    tf2
    tf2_ros
    tf2_msgs
    tf2_geometry_msgs
    spice_msgs
    spice_mapf_msgs
)

# python modules
ament_python_install_package(${PROJECT_NAME})

install(TARGETS
    mapf_navigator_node_cpp
    DESTINATION lib/${PROJECT_NAME})

# python nodes
install(PROGRAMS
  scripts/mapf_planner_node.py
  scripts/test_agent_node.py
  scripts/mapf_navigator_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
