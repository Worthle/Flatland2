// Copyright (c) 2017, Avidbots Corp.
// Copyright (c) 2026, Levent Soysal (Worthle).
// SPDX-License-Identifier: BSD-3-Clause
// Full license notices: LICENSE and SOURCE_NOTICES.

#include <flatland_plugins/update_timer.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/timekeeper.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#ifndef FLATLAND_PLUGINS_MODEL_TF_PUBLISHER_H
#define FLATLAND_PLUGINS_MODEL_TF_PUBLISHER_H

using namespace flatland_server;

namespace flatland_plugins {

/**
 * This class implements the model plugin and provides the functionality of
 * publishing ROS TF transformations for the bodies inside a model
 */
class ModelTfPublisher : public ModelPlugin {
 public:
  std::string world_frame_id_;  ///< name of the world frame id
  bool publish_tf_world_;  ///< if to publish the world position of the bodies
  std::vector<Body *> excluded_bodies_;  ///< list of bodies to ignore
  Body *reference_body_;  ///< body used as a reference to other bodies
  double update_rate_;    ///< publish rate

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;  ///< For publish ROS TF
  UpdateTimer update_timer_;                ///< for managing update rate

  /**
 * @brief Initialization for the plugin
 * @param[in] config Plugin YAML Node
 */
  void OnInitialize(const YAML::Node &config) override;

  /**
   * @brief Called when just before physics update
   * @param[in] timekeeper Object managing the simulation time
   */
  void BeforePhysicsStep(const Timekeeper &timekeeper) override;
};
};

#endif
