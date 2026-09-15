// Copyright (c) 2026, Levent Soysal (Worthle).
// SPDX-License-Identifier: BSD-3-Clause
// Full license notices: LICENSE and SOURCE_NOTICES.

#ifndef FLATLAND_PLUGINS_FORKLIFT_H
#define FLATLAND_PLUGINS_FORKLIFT_H

#include <flatland_msgs/srv/lift_fork.hpp>
#include <flatland_plugins/update_timer.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/types.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

#include <vector>

using namespace flatland_server;

namespace flatland_plugins {

/**
 * Forklift plugin: gives a model a set of "fork" bodies (e.g. the knives of an
 * a generic forklift) a lifted/lowered state. When lowered the forks sit on the ground and
 * are drawn in the lowered color (red by default); when lifted they are raised
 * in Z and drawn in the lifted color (green by default), giving a pseudo-3D
 * view in RViz. The target height is commanded via a flatland_msgs/LiftFork
 * service as a fraction of lift_height (0.0 = fully lowered, 1.0 = fully
 * lifted, e.g. 0.5 with lift_height 4.0 moves the forks to 2.0 m). The
 * state is continuously reported on a std_msgs/Bool topic (true = commanded
 * above fully-lowered) and the current fork height on a std_msgs/Float32
 * topic. With lift_speed > 0 the elevation is animated at that rate (m/s)
 * instead of jumping, and the color fades between the lowered and lifted
 * colors with the travel progress.
 *
 * Sibling plugins on the same model (e.g. ForkController) can resolve this
 * instance through LookupForModel() and command it directly with
 * SetTargetElevation() - same-process, same world-update thread.
 */
class Forklift : public ModelPlugin {
 public:
  std::vector<Body *> fork_bodies_;  ///< the fork bodies to control

  Color lowered_color_;   ///< color drawn while lowered
  Color lifted_color_;    ///< color drawn while lifted
  double lift_height_;    ///< Z elevation (m) of the forks when lifted
  double fork_thickness_;  ///< extrusion height (m) for the 3D fork rendering
  double lift_speed_;      ///< animation speed (m/s), <= 0 = instant

  bool lifted_ = false;  ///< current lift TARGET state (as commanded)
  double current_elevation_ = 0.0;  ///< animated elevation of the forks
  double target_elevation_ = 0.0;   ///< elevation the animation moves toward
  double update_rate_;   ///< rate to publish the state at

  UpdateTimer update_timer_;  ///< for managing the state publish rate

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr state_pub_;  ///< lift state
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr
      height_pub_;  ///< current fork elevation (m)
  rclcpp::Service<flatland_msgs::srv::LiftFork>::SharedPtr
      lift_srv_;  ///< fractional lift command

  /**
   * @brief Find the Forklift plugin instance attached to a model
   * @param[in] model the model to look up
   * @return the instance, or nullptr when the model has no Forklift plugin
   */
  static Forklift *LookupForModel(flatland_server::Model *model);

  /**
   * @brief Move the forks to an absolute elevation (for sibling plugins)
   * @param[in] elevation_m target elevation (m), clamped to [0, lift_height]
   */
  void SetTargetElevation(double elevation_m);

  /**
   * @brief Initialization for the plugin
   * @param[in] config Plugin YAML Node
   */
  void OnInitialize(const YAML::Node &config) override;

  /**
   * @brief Unregister this instance from the per-model registry
   */
  ~Forklift() override;

  /**
   * @brief Animate the fork elevation toward the commanded state
   * @param[in] timekeeper Object managing the simulation time
   */
  void BeforePhysicsStep(const Timekeeper &timekeeper) override;

  /**
   * @brief Publishes the current lift state at the configured update rate
   * @param[in] timekeeper Object managing the simulation time
   */
  void AfterPhysicsStep(const Timekeeper &timekeeper) override;

 private:
  /**
   * @brief Set the elevation on all fork bodies with a progress-faded color
   * @param[in] elevation the fork elevation (m) to apply
   */
  void ApplyElevation(double elevation);

  /**
   * @brief Publish the current lift state on the state topic
   */
  void PublishState();

  /**
   * @brief Service callback to move the forks to a fraction of lift_height
   */
  void OnLiftRequest(
      const std::shared_ptr<flatland_msgs::srv::LiftFork::Request> request,
      std::shared_ptr<flatland_msgs::srv::LiftFork::Response> response);
};
};  // namespace flatland_plugins

#endif  // FLATLAND_PLUGINS_FORKLIFT_H
