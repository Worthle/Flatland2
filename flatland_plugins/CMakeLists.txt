cmake_minimum_required(VERSION 3.8)
project(flatland_plugins)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(flatland_server REQUIRED)
find_package(flatland_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(OpenCV REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

set(dependencies
  ament_index_cpp
  rclcpp
  pluginlib
  flatland_server
  flatland_msgs
  std_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  ackermann_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_library(flatland_plugins_lib SHARED
  src/laser.cpp
  src/tricycle_drive.cpp
  src/tricycle_drive_ackermann.cpp
  src/diff_drive.cpp
  src/diff_drive_caster.cpp
  src/dynamics_limits.cpp
  src/imu.cpp
  src/model_tf_publisher.cpp
  src/update_timer.cpp
  src/bumper.cpp
  src/tween.cpp
  src/bool_sensor.cpp
  src/world_modifier.cpp
  src/world_random_wall.cpp
  src/gps.cpp
  src/omni_drive.cpp
  src/terrain_modifier.cpp
  src/forklift.cpp
  src/mock_detection.cpp
  src/mock_lidar3d.cpp
  src/link_attacher.cpp
  src/fork_controller.cpp
  src/narx_model.cpp
  src/system_id_drive.cpp
)
target_include_directories(flatland_plugins_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/thirdparty/tweeny>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(flatland_plugins_lib SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(flatland_plugins_lib ${dependencies})
target_link_libraries(flatland_plugins_lib
  ${Boost_LIBRARIES}
  yaml-cpp
)

# Plugin base classes (ModelPlugin / WorldPlugin) live in flatland_server
pluginlib_export_plugin_description_file(flatland_server flatland_plugins.xml)

# offline verification harness for the NARX system-id port: replays a
# recorded command csv so the C++ free-run can be diffed against the
# SystemID python reference (see system_id_drive.h)
add_executable(narx_sim_tool src/narx_sim_tool.cpp)
target_link_libraries(narx_sim_tool flatland_plugins_lib)

install(TARGETS flatland_plugins_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS narx_sim_tool
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(FILES flatland_plugins.xml LICENSE
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(flatland_plugins_lib)
ament_export_dependencies(${dependencies})
ament_package()
