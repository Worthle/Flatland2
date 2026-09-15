// Copyright (c) 2017, Avidbots Corp.
// Copyright (c) 2026, Levent Soysal (Worthle).
// SPDX-License-Identifier: BSD-3-Clause
// Full license notices: LICENSE and SOURCE_NOTICES.

#include <flatland_plugins/update_timer.h>
#include <flatland_server/model_plugin.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#ifndef FLATLAND_PLUGINS_BOOL_SENSOR_H
#define FLATLAND_PLUGINS_BOOL_SENSOR_H

using namespace flatland_server;

namespace flatland_plugins {

/**
 * This class defines a bumper plugin that is used to publish the collisions
 * states of bodies in the model
 */
class BoolSensor : public ModelPlugin {
 public:
  Body *body_;          ///< The body to check collisions with
  double update_rate_;  ///< rate to publish message at

  UpdateTimer update_timer_;    ///< for managing update rate
  int collisions_ = 0;          ///< Current number of collisions
  bool hit_something_ = false;  ///< "latch" var to ensure all hits published

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;  ///< For publishing the collisions

  /**
   * @brief Initialization for the plugin
   * @param[in] config Plugin YAML Node
   */
  void OnInitialize(const YAML::Node &config) override;

  /**
   * @brief Called when just after physics update to publish state
   * @param[in] timekeeper Object managing the simulation time
   */
  void AfterPhysicsStep(const Timekeeper &timekeeper) override;

  /**
   * @brief A method that is called for all Box2D begin contacts
   * @param[in] contact Box2D contact
   */
  void BeginContact(b2Contact *contact) override;

  /**
   * @brief A method that is called for all Box2D end contacts
   * @param[in] contact Box2D contact
   */
  void EndContact(b2Contact *contact) override;
};
};

#endif
