// Copyright (c) 2017, Avidbots Corp.
// Copyright (c) 2026, Levent Soysal (Worthle).
// SPDX-License-Identifier: BSD-3-Clause
// Full license notices: LICENSE.

#include <Box2D/Box2D.h>
#include <flatland_plugins/dynamics_limits.h>
#include <flatland_plugins/update_timer.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/timekeeper.h>
#include <tf2_ros/transform_broadcaster.h>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <flatland_msgs/msg/channel_values_floating.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <random>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <vector>

#ifndef FLATLAND_PLUGINS_OMNIDRIVE_H
#define FLATLAND_PLUGINS_OMNIDRIVE_H

using namespace flatland_server;

namespace flatland_plugins {

/**
 * @brief Structure to hold turret wheel data and state
 */
struct TurretWheel {
  // Configuration
  b2Vec2 pose;        ///< Position of wheel relative to body center [x, y]
  double base_angle;  ///< Base angle of wheel mount (radians)

  // Commanded values from AckermannDriveStamped
  double cmd_speed;     ///< Commanded speed (m/s)
  double cmd_steering;  ///< Commanded steering angle (radians)

  // Current state (after dynamics limits)
  double current_speed;     ///< Current speed after dynamics (m/s)
  double current_steering;  ///< Current steering angle after dynamics (radians)
  double steering_velocity;  ///< Current steering velocity (rad/s)

  // Dynamics limits
  DynamicsLimits linear_dynamics;    ///< Speed dynamics constraints
  DynamicsLimits steering_dynamics;  ///< Steering dynamics constraints
  double max_steer_angle;  ///< Maximum steering angle (radians), 0 = unlimited

  // Joint for visualization
  Joint* wheel_joint;    ///< Revolute joint for wheel steering visualization
  bool invert_steering;  ///< Whether to invert steering angle for joint

  TurretWheel()
      : pose(0, 0),
        base_angle(0),
        cmd_speed(0),
        cmd_steering(0),
        current_speed(0),
        current_steering(0),
        steering_velocity(0),
        max_steer_angle(0),
        wheel_joint(nullptr),
        invert_steering(false) {}
};

class OmniDrive : public flatland_server::ModelPlugin {
 public:
  // Subscribers for two turret wheels (AckermannDriveStamped)
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr
      turret1_sub_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr
      turret2_sub_;

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ground_truth_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      ground_truth_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
      twist_pub_;
  rclcpp::Publisher<flatland_msgs::msg::ChannelValuesFloating>::SharedPtr
      turret_angles_pub_;
  rclcpp::Publisher<flatland_msgs::msg::ChannelValuesFloating>::SharedPtr
      wrpms_pub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr
      turret1_cmd_pub;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr
      turret2_cmd_pub;
  Body* body_;

  // Two turret wheels
  TurretWheel turret1_;
  TurretWheel turret2_;
  double wheelbase_;  ///< Distance between the two turret wheels
  std::vector<b2RevoluteJoint*> caster_joints_;
  double caster_alignment_rate_ = 8.0;

  // Messages
  nav_msgs::msg::Odometry odom_msg_;
  nav_msgs::msg::Odometry ground_truth_msg_;
  geometry_msgs::msg::PoseWithCovarianceStamped pose_msg_;

  UpdateTimer update_timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster>
      tf_broadcaster;  ///< For publish ROS TF

  // Configuration flags
  bool enable_odom_pub_;       ///< YAML parameter to enable odom publishing
  bool enable_odom_tf_pub_;    ///< YAML parameter to enable odom tf publishing
  bool enable_twist_pub_;      ///< YAML parameter to enable twist publishing
  bool twist_in_local_frame_;  ///< YAML parameter to publish velocity in local
                               /// frame

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
   * @name        Turret1Callback
   * @brief       callback to apply ackermann command to turret 1 (front)
   * @param[in]   msg AckermannDriveStamped message with speed and steering
   * angle
   */
  void Turret1Callback(const ackermann_msgs::msg::AckermannDriveStamped& msg);

  /**
   * @name        Turret2Callback
   * @brief       callback to apply ackermann command to turret 2 (rear)
   * @param[in]   msg AckermannDriveStamped message with speed and steering
   * angle
   */
  void Turret2Callback(const ackermann_msgs::msg::AckermannDriveStamped& msg);

  /**
   * @name        ComputeTurretJoint
   * @brief       Validate and compute turret joint parameters
   * @param[in]   joint The joint to validate
   * @param[out]  turret The turret wheel struct to populate
   */
  void ComputeTurretJoint(Joint* joint, TurretWheel& turret);

  /**
   * @name        UpdateTurretState
   * @brief       Update turret wheel state with dynamics limits
   * @param[in/out] turret The turret wheel to update
   * @param[in]   dt Time step
   */
  void UpdateTurretState(TurretWheel& turret, double dt);

  /**
   * @name        ComputeBodyVelocity
   * @brief       Compute body velocity from two turret wheel states
   * @param[out]  vx Linear velocity in x (local frame)
   * @param[out]  vy Linear velocity in y (local frame)
   * @param[out]  omega Angular velocity
   */
  void ComputeBodyVelocity(double& vx, double& vy, double& omega);
  void UpdateCasters();
};  // class OmniDrive
}  // namespace flatland_plugins

#endif
