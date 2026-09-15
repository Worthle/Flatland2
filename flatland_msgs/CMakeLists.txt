cmake_minimum_required(VERSION 3.8)
project(flatland_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Vector2.msg"
  "msg/Collision.msg"
  "msg/Collisions.msg"
  "msg/DebugTopicList.msg"
  "msg/ChannelValuesFloating.msg"
  "srv/SpawnModel.srv"
  "srv/DeleteModel.srv"
  "srv/MoveModel.srv"
  "srv/Attach.srv"
  "srv/LiftFork.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
