cmake_minimum_required(VERSION 3.8)
project(flatland_server)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(flatland_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS date_time system filesystem)
find_package(Threads REQUIRED)
find_package(Lua 5.1 REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

set(dependencies
  rclcpp
  pluginlib
  std_msgs
  std_srvs
  geometry_msgs
  visualization_msgs
  rosgraph_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  interactive_markers
  flatland_msgs
)

# Bundled Box2D physics engine (compiled directly into flatland_lib)
file(GLOB_RECURSE BOX2D_SRCS thirdparty/Box2D/*.cpp)

add_library(flatland_lib SHARED
  ${BOX2D_SRCS}
  src/ros_node.cpp
  src/simulation_manager.cpp
  src/world.cpp
  src/layer.cpp
  src/model.cpp
  src/entity.cpp
  src/debug_visualization.cpp
  src/geometry.cpp
  src/body.cpp
  src/joint.cpp
  src/model_body.cpp
  src/collision_filter_registry.cpp
  src/model_plugin.cpp
  src/world_plugin.cpp
  src/plugin_manager.cpp
  src/interactive_marker_manager.cpp
  src/timekeeper.cpp
  src/service_manager.cpp
  src/yaml_reader.cpp
  src/dummy_model_plugin.cpp
  src/dummy_world_plugin.cpp
  src/yaml_preprocessor.cpp
)
target_include_directories(flatland_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(flatland_lib SYSTEM PUBLIC
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${LUA_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIRS}
)
ament_target_dependencies(flatland_lib ${dependencies})
target_link_libraries(flatland_lib
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
  ${LUA_LIBRARIES}
  Threads::Threads
  yaml-cpp
)

add_executable(flatland_server src/flatland_server_node.cpp)
ament_target_dependencies(flatland_server ${dependencies})
target_link_libraries(flatland_server flatland_lib)

# Register the dummy plugins (base classes live in this package)
pluginlib_export_plugin_description_file(flatland_server flatland_plugins.xml)

install(TARGETS flatland_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS flatland_server
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY thirdparty/Box2D/
  DESTINATION include/Box2D
  FILES_MATCHING PATTERN "*.h"
)
install(FILES flatland_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(flatland_lib)
ament_export_dependencies(${dependencies})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_visualization test/test_visualization.cpp)
  target_link_libraries(test_visualization flatland_lib)
endif()

ament_package()
