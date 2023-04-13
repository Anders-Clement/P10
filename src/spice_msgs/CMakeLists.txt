cmake_minimum_required(VERSION 3.8)
project(spice_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

set(msg_files
  "msg/LedOutput.msg"
  "msg/Id.msg"
  "msg/RobotState.msg"
  "msg/Robot.msg"
  "msg/RobotStateTransition.msg"
  "msg/Task.msg"
  "msg/Layer.msg"
  "msg/Node.msg"
  "msg/Work.msg"
  "msg/RobotType.msg"
  "msg/PlannerType.msg"
)

set(srv_files
  "srv/GetReadyRobots.srv"
  "srv/GetRobots.srv"
  "srv/Heartbeat.srv"
  "srv/RegisterRobot.srv"
  "srv/RobotTask.srv"
  "srv/AllocWorkCell.srv"
  "srv/RegisterWork.srv"
  "srv/GetRobotsByState.srv"
  "srv/GetRobotsByType.srv"
  "srv/GetPlan.srv"
  "srv/SetPlannerType.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}  
  DEPENDENCIES geometry_msgs nav_msgs
)

ament_export_dependencies(rosidl_default_runtime)


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

ament_package()
