// Copyright (c) 2017, Avidbots Corp.
// Copyright (c) 2026, Levent Soysal (Worthle).
// SPDX-License-Identifier: BSD-3-Clause
// Full license notices: LICENSE.

#include <Box2D/Box2D.h>
#include <flatland_plugins/discrete_oe_model.h>
#include <flatland_plugins/dynamics_limits.h>
#include <flatland_plugins/update_timer.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/timekeeper.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <random>

#ifndef FLATLAND_PLUGINS_DIFFDRIVE_H
#define FLATLAND_PLUGINS_DIFFDRIVE_H

using namespace flatland_server;

namespace flatland_plugins {

class DiffDrive : public flatland_server::ModelPlugin {
 public:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ground_truth_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      ground_truth_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
      twist_pub_;
  Body* body_;
  geometry_msgs::msg::Twist twist_msg_;
  nav_msgs::msg::Odometry odom_msg_;
  nav_msgs::msg::Odometry ground_truth_msg_;
  geometry_msgs::msg::PoseWithCovarianceStamped pose_msg_;
  UpdateTimer update_timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster>
      tf_broadcaster;          ///< For publish ROS TF
  bool enable_odom_pub_;       ///< YAML parameter to enable odom publishing
  bool enable_odom_tf_pub_;    ///< YAML parameter to enable odom tf publishing
  bool enable_twist_pub_;      ///< YAML parameter to enable twist publishing
  bool twist_in_local_frame_;  ///< YAML parameter to publish velocity in local
                               /// frame. Original diff drive plugin publishes
                               /// local velocity wrt to odom frame
  DynamicsLimits angular_dynamics_;  ///< Angular dynamics constraints
  DynamicsLimits linear_dynamics_;   ///< Linear dynamics constraints
  double angular_velocity_ = 0.0;
  double linear_velocity_ = 0.0;

  // Identified OE models
  bool use_id_model_ = false;         ///< YAML flag: use identified OE models
  DiscreteOEModel linear_oe_model_;   ///< OE model: cmd_v -> odom linear vel
  DiscreteOEModel angular_oe_model_;  ///< OE model: cmd_w -> odom angular vel
  double oe_time_accumulator_ =
      0.0;  ///< Accumulates physics dt for OE sample stepping
  double oe_linear_output_ = 0.0;   ///< Latest OE linear model output
  double oe_angular_output_ = 0.0;  ///< Latest OE angular model output

  // Initial pose
  b2Vec2 initial_position_;
  float initial_angle_;
  bool initialized_ = false;

  std::default_random_engine rng_;
  std::array<std::normal_distribution<double>, 6> noise_gen_;

  /**
   * @name          OnInitialize
   * @brief         override the BeforePhysicsStep method
   * @param[in]     config The plugin YAML node
   */
  void OnInitialize(const YAML::Node& config) override;
  /**
   * @name          BeforePhysicsStep
   * @brief         override the BeforePhysicsStep method
   * @param[in]     config The plugin YAML node
   */
  void BeforePhysicsStep(const Timekeeper& timekeeper) override;
  /**
   * @name        TwistCallback
   * @brief       callback to apply twist (velocity and omega)
   * @param[in]   timestep how much the physics time will increment
   */
  void TwistCallback(const geometry_msgs::msg::Twist& msg);
};
};

#endif
