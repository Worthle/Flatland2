#ifndef FLATLAND_SERVER_ROS_NODE_H
#define FLATLAND_SERVER_ROS_NODE_H

#include <rclcpp/rclcpp.hpp>

namespace flatland_server {

/**
 * @brief Accessor for the single, global flatland rclcpp node.
 *
 * ROS 1 provided a process-global node via default-constructed
 * ros::NodeHandle. ROS 2 has no such global, so the flatland server creates a
 * single node in main() and stores it here. All flatland classes (timekeeper,
 * services, debug visualization, plugins, ...) retrieve it through this
 * accessor, preserving the original architecture with a minimal diff.
 *
 * The reference must be assigned exactly once, early in main(), before any
 * flatland object is constructed.
 */
rclcpp::Node::SharedPtr& ros_node();

}  // namespace flatland_server

#endif  // FLATLAND_SERVER_ROS_NODE_H
