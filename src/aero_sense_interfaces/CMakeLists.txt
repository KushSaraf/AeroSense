cmake_minimum_required(VERSION 3.8)
project(aero_sense_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TriageScore.msg"
  "msg/VictimDetection.msg"
  "msg/VictimArray.msg"
  "msg/HazardPolygon.msg"
  "msg/HazardDetection.msg"
  "msg/HazardArray.msg"
  "msg/Alert.msg"
  "msg/DroneStatus.msg"
  "msg/MissionStatus.msg"
  "msg/SafeRoute.msg"
  "msg/SafeRouteArray.msg"
  "msg/CommunicationStatus.msg"
  "msg/LocalizationStatus.msg"
  "srv/StartMission.srv"
  "srv/SetSearchArea.srv"
  "srv/SetPriority.srv"
  "srv/InjectFailure.srv"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs nav_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
