// Copyright (c) 2026, Levent Soysal (Worthle).
// SPDX-License-Identifier: BSD-3-Clause
// Full license notices: LICENSE.

#ifndef FLATLAND_PLUGINS_FORK_CONTROLLER_H
#define FLATLAND_PLUGINS_FORK_CONTROLLER_H

#include <flatland_plugins/update_timer.h>
#include <flatland_server/model_plugin.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>

#include <string>

using namespace flatland_server;

namespace flatland_plugins {

class Forklift;
class LinkAttacher;

/**
 * Generic fork height controller with load/contact feedback and optional
 * automatic pickup/dropoff. Requires Forklift and LinkAttacher on the model.
 * Height commands use metres. Contact signals use capture-point proximity;
 * weight is a configured mock value. Fork-tip collision is not modeled.
 */
class ForkController : public ModelPlugin {
 public:
  double goal_tolerance_;   ///< |height-goal| for goal_completed (m)
  double contact_range_;    ///< capture-point distance reading "seated" (m)
  double loaded_weight_;    ///< fork/weight (kg) while a model is attached
  bool auto_attach_;        ///< weld/release on lift/lower crossings
  double attach_height_;    ///< rising past this attaches the candidate (m)
  double detach_height_;    ///< falling below this releases the load (m)
  double update_rate_;      ///< publish rate (Hz)
  std::string pose_frame_;  ///< frame_id stamped on fork/pose

  bool has_goal_ = false;         ///< a fork goal has been received
  double goal_z_ = 0.0;           ///< last commanded height (m, clamped)
  double prev_elevation_ = 0.0;   ///< previous-step fork height (m)
  bool warned_no_forklift_ = false;
  bool warned_no_attacher_ = false;

  UpdateTimer update_timer_;

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr goal_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr completed_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr weight_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr contact_left_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr contact_right_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr collision_left_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr collision_right_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr load_state_pub_;

  /**
   * @brief Initialization for the plugin
   * @param[in] config Plugin YAML Node
   */
  void OnInitialize(const YAML::Node &config) override;

  /**
   * @brief Auto attach/detach on height crossings and publish the contract
   * @param[in] timekeeper Object managing the simulation time
   */
  void BeforePhysicsStep(const Timekeeper &timekeeper) override;

 private:
  /**
   * @brief Resolve the sibling Forklift plugin, warning once when missing
   */
  Forklift *GetForklift();

  /**
   * @brief Resolve the sibling LinkAttacher plugin, warning once when missing
   */
  LinkAttacher *GetAttacher();

  /**
   * @brief Fork goal callback: clamp to the stroke and command the Forklift
   */
  void OnForkGoal(const std_msgs::msg::Float32::SharedPtr msg);
};
};  // namespace flatland_plugins

#endif  // FLATLAND_PLUGINS_FORK_CONTROLLER_H
