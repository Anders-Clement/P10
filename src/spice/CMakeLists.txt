cmake_minimum_required(VERSION 3.8)
project(spice)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED) 
find_package(spice_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(robot_tf_relay src/FramePublisher.cpp)
ament_target_dependencies(
    robot_tf_relay
    geometry_msgs
    rclcpp
    tf2
    tf2_ros
    tf2_msgs
)

add_executable(robot_tf_pose src/TFRelay.cpp)
ament_target_dependencies(
    robot_tf_pose
    geometry_msgs
    rclcpp
    tf2
    tf2_ros
    tf2_msgs
)

add_executable(task_allocator src/TaskAllocator2.cpp)
ament_target_dependencies(
    task_allocator
    geometry_msgs
    rclcpp
    spice_msgs
)


add_executable(set_robot_task_poses src/SetRobotTaskPoses.cpp)
ament_target_dependencies(
  set_robot_task_poses
  geometry_msgs
  rclcpp
  visualization_msgs
)

install(TARGETS
    robot_tf_relay
    robot_tf_pose
    task_allocator
    set_robot_task_poses
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME})

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

# # Install Python modules
# ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  scripts/led_node.py
  scripts/SwarmManager.py
  scripts/robot_state_manager_node.py
  scripts/dynamic_obstacle_avoidance.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
