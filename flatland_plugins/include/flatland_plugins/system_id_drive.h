// Copyright (c) 2017, Avidbots Corp.
// Copyright (c) 2026, Levent Soysal (Worthle).
// SPDX-License-Identifier: BSD-3-Clause
// Full license notices: LICENSE.

#ifndef FLATLAND_PLUGINS_SYSTEM_ID_DRIVE_H
#define FLATLAND_PLUGINS_SYSTEM_ID_DRIVE_H

#include <flatland_plugins/narx_model.h>
#include <flatland_plugins/update_timer.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/types.h>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <random>
#include <string>
#include <vector>

using namespace flatland_server;

namespace flatland_plugins {

class SystemIdDrive : public ModelPlugin {
 public:
  Body *body_;                 ///< body driven by the identified model
  NarxCoupledModel model_;     ///< the identified NARX model (coupled free-run)
  std::string linear_output_;  ///< model output used as body v [m/s]
  std::string angular_output_;  ///< model output used as body w [rad/s]
  int linear_index_ = -1;       ///< index of linear_output_ in Step() result
  int angular_index_ = -1;      ///< index of angular_output_ in Step() result
  std::vector<int> command_fields_;  ///< per model command: message field id

  double cmd_timeout_;  ///< [s] without a command -> feed zeros (0 = hold)
  double v_hat_ = 0.0;  ///< latest predicted linear velocity
  double w_hat_ = 0.0;  ///< latest predicted angular velocity

  double cmd_speed_ = 0.0;        ///< latest received command: drive.speed
  double cmd_steering_ = 0.0;     ///< latest received command: steering_angle
  bool cmd_received_ = false;     ///< a new command arrived since the last step
  double last_cmd_sim_s_ = -1.0;  ///< sim time [s] of last command, -1 = none

  UpdateTimer model_timer_;  ///< NARX tick at the model's sample rate
  UpdateTimer pub_timer_;    ///< odom publish rate

  bool initialized_ = false;  ///< odom origin latched on first step
  b2Vec2 initial_position_;   ///< world position at start (odom origin)
  float initial_angle_;       ///< world angle at start (odom origin)

  geometry_msgs::msg::PoseWithCovarianceStamped pose_msg_;
  nav_msgs::msg::Odometry odom_msg_;
  nav_msgs::msg::Odometry ground_truth_msg_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr
      command_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ground_truth_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      ground_truth_pose_pub_;

  std::default_random_engine rng_;
  std::array<std::normal_distribution<double>, 6> noise_gen_;

  /**
   * @brief Initialization for the plugin: parses params and loads the
   * identified model file, failing loudly on any problem
   * @param[in] config Plugin YAML Node
   */
  void OnInitialize(const YAML::Node &config) override;

  /**
   * @brief Tick the NARX model at its native rate and write the predicted
   * velocities onto the Box2D body every timestep; publish odometry
   * @param[in] timekeeper Object managing the simulation time
   */
  void BeforePhysicsStep(const Timekeeper &timekeeper) override;

 private:
  /**
   * @brief Command callback storing the latest command signals
   */
  void CommandCallback(const ackermann_msgs::msg::AckermannDriveStamped &msg);
};

}  // namespace flatland_plugins

#endif  // FLATLAND_PLUGINS_SYSTEM_ID_DRIVE_H
