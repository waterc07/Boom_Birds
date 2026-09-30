cmake_minimum_required(VERSION 3.8)
project(boom_birds_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ControlCommand.msg" "msg/ExecutionStatus.msg" "msg/PlannerRequest.msg" "msg/PlannerStatus.msg"
  "srv/VehicleAction.srv" "srv/Mission.srv"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces)
ament_export_dependencies(rosidl_default_runtime)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
ament_package()
