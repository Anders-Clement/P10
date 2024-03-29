cmake_minimum_required(VERSION 3.8)
project(spice)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
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
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)

include_directories(
  include
)

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

add_executable(task_allocator src/TaskAllocator.cpp)
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

add_executable(work_cell_allocator src/WorkCellAllocator.cpp)
ament_target_dependencies(
  work_cell_allocator
  geometry_msgs
  rclcpp
  spice_msgs
  tf2
  tf2_ros
  tf2_msgs
)
set(WORK_CELL_SIMULATOR_SOURCES 
  src/work_cells/WorkCell.cpp
  src/work_cells/WorkCellSimulator.cpp
  src/work_cells/WorkCellStateMachine.cpp
  src/work_cells/QueueManager.cpp
  src/work_cells/WorkCellQueuePositionManager.cpp
)
add_executable(work_cell_simulator ${WORK_CELL_SIMULATOR_SOURCES})
ament_target_dependencies(
  work_cell_simulator
  rclcpp
  spice_msgs
  std_srvs
  geometry_msgs
  tf2_ros
  nav2_costmap_2d
  nav_msgs
)

set(CENTRAL_PLANNER_SOURCES
  src/central_path_planner.cpp
  src/planners/global_planner.cpp
  src/planners/straight_line_planner.cpp
  src/planners/a_star_planner.cpp
  src/planners/navfn.cpp
  src/costmaps/prioritized_costmap.cpp
  src/costmaps/costmap.cpp
  src/costmaps/global_costmap.cpp
  )

add_executable(central_path_planner ${CENTRAL_PLANNER_SOURCES})
ament_target_dependencies(
  central_path_planner
  rclcpp
  nav_msgs
  spice_msgs
  nav2_costmap_2d
  tf2
  tf2_ros
  tf2_msgs
)

install(TARGETS
    robot_tf_relay
    robot_tf_pose
    task_allocator
    work_cell_simulator
    work_cell_allocator
    set_robot_task_poses
    central_path_planner
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
  scripts/work_tree.py
  scripts/gui.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
