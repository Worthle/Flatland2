// Copyright (c) 2026, Levent Soysal (Worthle).
// SPDX-License-Identifier: BSD-3-Clause
// Full license notices: LICENSE.

#ifndef FLATLAND_PLUGINS_LINK_ATTACHER_H
#define FLATLAND_PLUGINS_LINK_ATTACHER_H

#include <Box2D/Box2D.h>
#include <flatland_msgs/srv/attach.hpp>
#include <flatland_plugins/update_timer.h>
#include <flatland_server/model.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/types.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <string>
#include <vector>

using namespace flatland_server;

namespace flatland_plugins {

/**
 * Runtime attach/detach of another model to this model (a gazebo
 * link-attacher equivalent): the attach service welds a target model (named
 * explicitly, or the nearest model matching the configured name prefixes
 * within capture range of the capture point) to the configured body with a
 * rigid b2WeldJoint, so it moves with the robot. While attached the two
 * models stop colliding with each other (fixture group filtering; sensors
 * still see the carried model), and the carried model's bodies copy the
 * elevation of the configured fork body (so a lifted pallet rides the forks
 * up and down in the pseudo-3D view). The detach service (std_srvs/Trigger)
 * removes the weld, restores collisions and grounds the carried model's
 * elevation.
 */
class LinkAttacher : public ModelPlugin {
 public:
  Body *attach_body_;             ///< body the weld joint anchors to
  Body *elevation_source_body_;   ///< fork body carried models copy, or null
  std::vector<std::string> model_prefixes_;  ///< attachable model prefixes
  b2Vec2 capture_point_;   ///< auto-select reference point (body frame)
  double capture_range_;   ///< max distance from capture point to target
  double carry_elevation_offset_;  ///< added to the copied fork elevation
  double drop_speed_;   ///< settle speed (m/s) of a detached lifted model
  double update_rate_;  ///< rate to publish the attached state at

  std::string attached_model_name_;  ///< name of the welded model, or empty
  std::string settling_model_;  ///< detached model still dropping to ground
  b2Joint *joint_ = nullptr;    ///< the runtime weld joint
  int nocollide_group_;  ///< fixture group suppressing robot<->target contact

  UpdateTimer update_timer_;  ///< for managing the state publish rate
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
      state_pub_;  ///< attached model name ('' = none)
  rclcpp::Service<flatland_msgs::srv::Attach>::SharedPtr attach_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr detach_srv_;

  /**
   * @brief Find the LinkAttacher plugin instance attached to a model
   * @param[in] model the model to look up
   * @return the instance, or nullptr when the model has no LinkAttacher
   */
  static LinkAttacher *LookupForModel(flatland_server::Model *model);

  /**
   * @brief Initialization for the plugin
   * @param[in] config Plugin YAML Node
   */
  void OnInitialize(const YAML::Node &config) override;

  /**
   * @brief Follow the fork elevation and survive external target deletion
   * @param[in] timekeeper Object managing the simulation time
   */
  void BeforePhysicsStep(const Timekeeper &timekeeper) override;

  /**
   * @brief Clean up the weld joint and collision filters on plugin removal
   */
  ~LinkAttacher() override;

  /**
   * @brief Whether a model is currently welded on
   */
  bool IsAttached() const { return !attached_model_name_.empty(); }

  /**
   * @brief Weld a model to the attach body (also for sibling plugins)
   * @param[in] model_name the model to attach, '' = nearest candidate
   * @param[out] message human readable result
   * @return true on success
   */
  bool Attach(const std::string &model_name, std::string &message);

  /**
   * @brief Detach the currently attached model, if any
   * @param[out] message human readable result
   * @return true on success
   */
  bool Detach(std::string &message);

  /**
   * @brief Find the nearest prefix-matching model within capture range
   * @param[out] message failure reason when nullptr is returned
   * @param[out] distance distance (m) from the capture point, when found
   * @return the selected model, or nullptr
   */
  Model *FindNearestCandidate(std::string &message,
                              double *distance = nullptr);

 private:
  /**
   * @brief Find a model in the physics world by its name
   * @param[in] name the model name
   * @return the model, or nullptr if it does not exist
   */
  Model *FindModelByName(const std::string &name);

 /**
   * @brief Set the collision group on every fixture of every body of a model
   * @param[in] model the model to refilter
   * @param[in] group the b2 group index to apply (0 restores the default)
   */
  void SetCollisionGroup(Model *model, int group);

  /**
   * @brief Set the elevation of every body of a model
   * @param[in] model the model
   * @param[in] elevation the elevation (m) to apply
   */
  void SetModelElevation(Model *model, double elevation);

  /**
   * @brief Attach service: weld the named (or nearest) model to this model
   */
  void OnAttachRequest(
      const std::shared_ptr<flatland_msgs::srv::Attach::Request> request,
      std::shared_ptr<flatland_msgs::srv::Attach::Response> response);

  /**
   * @brief Detach service: remove the weld and restore collisions
   */
  void OnDetachRequest(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
};
};  // namespace flatland_plugins

#endif  // FLATLAND_PLUGINS_LINK_ATTACHER_H
