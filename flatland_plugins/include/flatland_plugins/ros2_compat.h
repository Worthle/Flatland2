// Copyright (c) 2026, Levent Soysal (Worthle).
// SPDX-License-Identifier: BSD-3-Clause
// Full license notices: LICENSE.

#ifndef FLATLAND_PLUGINS_ROS2_COMPAT_H
#define FLATLAND_PLUGINS_ROS2_COMPAT_H

#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <string>

namespace flatland_plugins {

/**
 * @brief ROS 1 tf::resolve replacement. ROS 2 has no tf_prefix, so this simply
 * joins an (optional) prefix with a frame id and strips leading slashes.
 */
inline std::string resolveTf(const std::string &prefix,
                             const std::string &frame) {
  std::string f = frame;
  while (!f.empty() && f.front() == '/') f.erase(f.begin());
  if (prefix.empty()) return f;
  std::string p = prefix;
  while (!p.empty() && p.front() == '/') p.erase(p.begin());
  return p + "/" + f;
}

/**
 * @brief ROS 1 tf::createQuaternionMsgFromYaw replacement.
 */
inline geometry_msgs::msg::Quaternion quaternionMsgFromYaw(double yaw) {
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return tf2::toMsg(q);
}

}  // namespace flatland_plugins

#endif  // FLATLAND_PLUGINS_ROS2_COMPAT_H
