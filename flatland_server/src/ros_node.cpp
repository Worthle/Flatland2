#include <flatland_server/ros_node.h>

namespace flatland_server {

rclcpp::Node::SharedPtr & ros_node() {
  static rclcpp::Node::SharedPtr node;
  return node;
}

}  // namespace flatland_server
