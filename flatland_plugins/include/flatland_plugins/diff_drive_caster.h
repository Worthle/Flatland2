// Copyright (c) 2017, Avidbots Corp.
// Copyright (c) 2026, Levent Soysal (Worthle).
// SPDX-License-Identifier: BSD-3-Clause
// Full license notices: LICENSE and SOURCE_NOTICES.

#ifndef FLATLAND_PLUGINS_DIFF_DRIVE_CASTER_H
#define FLATLAND_PLUGINS_DIFF_DRIVE_CASTER_H

#include <Box2D/Box2D.h>
#include <flatland_plugins/dynamics_limits.h>
#include <flatland_plugins/update_timer.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/timekeeper.h>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <random>
#include <string>
#include <vector>

using namespace flatland_server;

namespace flatland_plugins {

/**
 * @brief Caster wheel configuration
 */
struct CasterConfig {
  std::string body_name;   // Name of the caster body in the model
  Body* body;              // Pointer to the caster body (resolved at init)
  b2Vec2 contact_offset;   // Wheel contact point offset in caster body frame
};

/**
 * @brief Differential drive with approximate caster constraints
 */
class DiffDriveCaster : public flatland_server::ModelPlugin {
 public:
  // ROS communication
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ground_truth_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr ground_truth_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_pub_;

  // Base body reference
  Body* body_;

  // Wheel configurations
  std::vector<CasterConfig> casters_;

  // Message storage
  geometry_msgs::msg::Twist twist_msg_;
  nav_msgs::msg::Odometry odom_msg_;
  nav_msgs::msg::Odometry ground_truth_msg_;
  geometry_msgs::msg::PoseWithCovarianceStamped pose_msg_;

  // Publishing
  UpdateTimer update_timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

  // Configuration flags
  bool enable_odom_pub_;
  bool enable_odom_tf_pub_;
  bool enable_twist_pub_;
  bool twist_in_local_frame_;

  // Dynamics constraints
  DynamicsLimits angular_dynamics_;
  DynamicsLimits linear_dynamics_;

  // Current velocity command (after dynamics limiting)
  double angular_velocity_cmd_ = 0.0;
  double linear_velocity_cmd_ = 0.0;

  // Initial pose for odometry reference
  b2Vec2 initial_position_;
  float initial_angle_;
  bool initialized_ = false;

  // Noise generation
  std::default_random_engine rng_;
  std::array<std::normal_distribution<double>, 6> noise_gen_;

  // --- Caster friction parameters ---
  // caster_lat_mu: 0 = full lateral constraint (no sideways slip)
  //                1 = no lateral constraint (free sliding)
  double caster_lat_mu_;
  double caster_alignment_rate_ = 8.0;
  
  // caster_long_mu: 0 = no rolling resistance
  //                 1 = full longitudinal constraint (no rolling)
  double caster_long_mu_;
  

  /**
   * @brief Initialize the plugin from YAML configuration
   */
  void OnInitialize(const YAML::Node& config) override;

  /**
   * @brief Called before each physics step
   */
  void BeforePhysicsStep(const Timekeeper& timekeeper) override;

  /**
   * @brief Callback for velocity command messages
   */
  void TwistCallback(const geometry_msgs::msg::Twist& msg);
};

}  // namespace flatland_plugins

#endif  // FLATLAND_PLUGINS_DIFF_DRIVE_CASTER_H
