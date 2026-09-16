// Copyright (c) 2017, Avidbots Corp.
// Copyright (c) 2026, Levent Soysal (Worthle).
// SPDX-License-Identifier: BSD-3-Clause
// Full license notices: LICENSE.

#ifndef FLATLAND_PLUGINS_MOCK_DETECTION_H
#define FLATLAND_PLUGINS_MOCK_DETECTION_H

#include <flatland_plugins/update_timer.h>
#include <flatland_server/model.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/types.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <map>
#include <random>
#include <string>
#include <vector>

using namespace flatland_server;

namespace flatland_plugins {

/**
 * Publishes PoseArray detections for matching model-name prefixes within a
 * camera field of view and range, subject to raycast occlusion. Camera-frame
 * poses use +X forward with configurable pose noise and frame dropout.
 * No message is published when no targets are detected.
 */
class MockDetection : public ModelPlugin {
 public:
  std::string topic_;         ///< topic to publish the PoseArray detections on
  Body *body_;                ///< body the camera is mounted on
  Pose origin_;               ///< camera mount pose w.r.t the body
  Pose pose_offset_;          ///< offset applied to each target pose, in the
                              ///< target's own frame (model origin -> face)
  std::string frame_id_;      ///< camera frame id
  bool broadcast_tf_;         ///< whether to broadcast camera origin w.r.t body
  bool broadcast_target_tf_;  ///< broadcast per-target anchored frames
  std::string target_frame_prefix_;  ///< prefix of the anchored target frames
  std::string world_frame_;          ///< world frame anchoring the targets
  double update_rate_;               ///< detection update rate (Hz)
  double min_range_;                 ///< minimum detection distance (m)
  double max_range_;                 ///< maximum detection distance (m)
  double fov_;                       ///< full horizontal field of view (rad)
  double max_target_elevation_;  ///< targets elevated above this (m) are out
                                 ///< of the camera's vertical view
  std::vector<std::string> model_prefixes_;  ///< model name prefixes to detect
  uint16_t layers_bits_;        ///< layers whose fixtures occlude the view
  double dropout_probability_;  ///< probability of missing a whole frame

  std::default_random_engine rng_;                      ///< random generator
  std::normal_distribution<double> noise_gen_x_;        ///< noise on x
  std::normal_distribution<double> noise_gen_y_;        ///< noise on y
  std::normal_distribution<double> noise_gen_yaw_;      ///< noise on yaw
  std::uniform_real_distribution<double> dropout_gen_;  ///< frame dropout roll

  UpdateTimer update_timer_;  ///< for controlling the update rate
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr
      detections_publisher_;  ///< PoseArray detections publisher
  std::shared_ptr<tf2_ros::TransformBroadcaster>
      tf_broadcaster_;                              ///< broadcast camera frame
  geometry_msgs::msg::TransformStamped camera_tf_;  ///< body to camera frame
  std::string resolved_frame_id_;  ///< namespaced frame id used in headers

  /// last-seen world-frame pose per detected model, keyed by model name;
  /// rebroadcast every update so lost targets stay anchored in the world
  /// frame instead of riding along with the robot
  std::map<std::string, geometry_msgs::msg::TransformStamped> anchored_tfs_;

  /**
   * @brief Initialization for the plugin
   * @param[in] config Plugin YAML Node
   */
  void OnInitialize(const YAML::Node &config) override;

  /**
   * @brief Detect visible targets and publish the noisy PoseArray
   * @param[in] timekeeper Object managing the simulation time
   */
  void BeforePhysicsStep(const Timekeeper &timekeeper) override;

 private:
  /**
   * @brief helper function to extract the parameters from the YAML Node
   * @param[in] config Plugin YAML Node
   */
  void ParseParameters(const YAML::Node &config);

  /**
   * @brief Raycast line-of-sight check from the camera to the target
   * @param[in] sensor_pos camera position in the world frame
   * @param[in] target the candidate target model
   * @param[in] target_pos target position in the world frame
   * @return true if nothing (other than the target itself) blocks the view
   */
  bool TargetVisible(const b2Vec2 &sensor_pos, Model *target,
                     const b2Vec2 &target_pos);

  /**
   * @brief Rebroadcast all anchored (last-seen) target frames
   * @param[in] stamp sim time to stamp the transforms with
   */
  void BroadcastAnchoredTfs(const rclcpp::Time &stamp);
};

/**
 * This class handles the b2RayCastCallback ReportFixture method for the
 * occlusion check, keeping the closest non-ignored hit along the ray
 */
class DetectionRayCallback : public b2RayCastCallback {
 public:
  MockDetection *parent_;             ///< the parent MockDetection plugin
  Model *target_;                     ///< the model being checked
  Entity *closest_entity_ = nullptr;  ///< entity of the closest hit, if any
  float closest_fraction_ = 1.0f;     ///< ray fraction of the closest hit

  /**
   * Default constructor to assign parent object and target model
   */
  DetectionRayCallback(MockDetection *parent, Model *target)
      : parent_(parent), target_(target) {}

  /**
   * @brief Box2D raytrace call back method required for implementing the
   * b2RayCastCallback abstract class
   * @param[in] fixture Fixture the ray hits
   * @param[in] point Point the ray hits the fixture
   * @param[in] normal Vector indicating the normal at the point hit
   * @param[in] fraction Fraction of ray length at hit point
   */
  float ReportFixture(b2Fixture *fixture, const b2Vec2 &point,
                      const b2Vec2 &normal, float fraction) override;
};
};  // namespace flatland_plugins

#endif  // FLATLAND_PLUGINS_MOCK_DETECTION_H
