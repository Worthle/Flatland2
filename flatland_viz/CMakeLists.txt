cmake_minimum_required(VERSION 3.8)
project(flatland_viz)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(flatland_server REQUIRED)
find_package(flatland_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)
find_package(Boost REQUIRED COMPONENTS filesystem)

set(dependencies
  rclcpp
  pluginlib
  rviz_common
  rviz_rendering
  rviz_default_plugins
  flatland_server
  flatland_msgs
  std_srvs
  geometry_msgs
)

# rviz tool plugins (loaded by any rviz2 instance, incl. our flatland_viz exe)
add_library(flatland_viz_tools SHARED
  src/spawn_model_tool.cpp
  src/pause_sim_tool.cpp
  src/load_model_dialog.cpp
  include/flatland_viz/spawn_model_tool.h
  include/flatland_viz/pause_sim_tool.h
  include/flatland_viz/load_model_dialog.h
)
target_include_directories(flatland_viz_tools PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(flatland_viz_tools ${dependencies})
target_link_libraries(flatland_viz_tools
  Qt5::Core Qt5::Gui Qt5::Widgets
  rviz_ogre_vendor::OgreMain
  ${Boost_LIBRARIES}
)
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

# flatland_viz application
add_executable(flatland_viz src/flatland_viz_node.cpp)
ament_target_dependencies(flatland_viz ${dependencies})
target_link_libraries(flatland_viz
  Qt5::Core Qt5::Gui Qt5::Widgets
  flatland_viz_tools
)

install(TARGETS flatland_viz_tools
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS flatland_viz
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()
