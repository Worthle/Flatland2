cmake_minimum_required(VERSION 3.8)
project(flatland)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch robot objects maps rviz config docs tools tests
  DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/diff_teleop.py scripts/ackermann_teleop.py scripts/omni_teleop.py
  scripts/odometry_tf.py scripts/omni_command.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/teleop_support.py DESTINATION lib/${PROJECT_NAME})
install(FILES LICENSE DESTINATION share/${PROJECT_NAME})
ament_package()
