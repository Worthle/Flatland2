// Copyright (c) 2017, Avidbots Corp.
// Copyright (c) 2026, Levent Soysal (Worthle).
// SPDX-License-Identifier: BSD-3-Clause
// Full license notices: LICENSE.

#include <Box2D/Box2D.h>
#include <flatland_plugins/update_timer.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/timekeeper.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/twist.hpp>
#include <random>
#include <sensor_msgs/msg/imu.hpp>

#ifndef FLATLAND_PLUGINS_IMU_H
#define FLATLAND_PLUGINS_IMU_H

using namespace flatland_server;

namespace flatland_plugins {

class Imu : public flatland_server::ModelPlugin {
 public:
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr ground_truth_pub_;
  Body* body_;
  sensor_msgs::msg::Imu imu_msg_;
  sensor_msgs::msg::Imu ground_truth_msg_;
  UpdateTimer update_timer_;
  // realized-rotation gyro: yaw sampled at the previous publish
  double last_angle_ = 0.0;
  double last_angle_time_ = 0.0;
  bool have_last_angle_ = false;
  bool enable_imu_pub_;  ///< YAML parameter to enable odom publishing

  std::default_random_engine rng_;
  std::array<std::normal_distribution<double>, 9> noise_gen_;
  geometry_msgs::msg::TransformStamped imu_tf_;  ///< tf from body to IMU frame
  std::shared_ptr<tf2_ros::TransformBroadcaster>
      tf_broadcaster_;  ///< broadcast IMU frame
  std::string imu_frame_id_;
  bool broadcast_tf_;
  b2Vec2 linear_vel_local_prev;
  double pub_rate_;
  /**
   * @name          OnInitialize
   * @brief         override the BeforePhysicsStep method
   * @param[in]     config The plugin YAML node
   */
  void OnInitialize(const YAML::Node& config) override;
  /**
   * @name          AfterPhysicsStep
   * @brief         override the AfterPhysicsStep method
   * @param[in]     timekeeper Tracks time in flatland
   */
  void AfterPhysicsStep(const Timekeeper& timekeeper) override;
  /**
   * @name        TwistCallback
   * @brief       callback to apply twist (velocity and omega)
   * @param[in]   timestep how much the physics time will increment
   */
  void TwistCallback(const geometry_msgs::msg::Twist& msg);
};
};

#endif
