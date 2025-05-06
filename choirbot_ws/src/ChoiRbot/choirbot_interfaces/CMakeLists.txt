cmake_minimum_required(VERSION 3.5)
project(choirbot_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/PositionAction.action"
  "msg/PositionTask.msg"
  "msg/PositionTaskArray.msg"
  "srv/PositionTaskService.srv"
  "srv/TaskCompletionService.srv"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
