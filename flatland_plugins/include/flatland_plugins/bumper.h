// Copyright (c) 2017, Avidbots Corp.
// Copyright (c) 2026, Levent Soysal (Worthle).
// SPDX-License-Identifier: BSD-3-Clause
// Full license notices: LICENSE and SOURCE_NOTICES.

#include <flatland_plugins/update_timer.h>
#include <flatland_server/model_plugin.h>
#include <flatland_msgs/msg/collisions.hpp>
#include <rclcpp/rclcpp.hpp>

#ifndef FLATLAND_PLUGINS_BUMPER_H
#define FLATLAND_PLUGINS_BUMPER_H

using namespace flatland_server;

namespace flatland_plugins {

/**
 * This class defines a bumper plugin that is used to publish the collisions
 * states of bodies in the model
 */
class Bumper : public ModelPlugin {
 public:
  struct ContactState {
    int num_count;  ///< stores number of times post solve is called
    double sum_normal_impulses[2];      ///< sum of impulses for averaging later
    double sum_tangential_impulses[2];  ///< sum of impulses for averaging later
    b2Vec2 points[2];  ///< Box2D collision points, max of 2 from Box2D
    b2Vec2 normal;  ///< normal of collision points, all points have same normal
    int normal_sign;  ///< for flipping direction of normal when necessary

    Body *body_A;      ///< the body of the model involved in the collision
    Body *body_B;      ///< the other body involved in the collision
    Entity *entity_B;  /// the entity the other body belongs to

    ContactState();  ///< initializes counters and sums
    void Reset();    ///< Reset counter and sums
  };

  std::string topic_name_;
  std::string world_frame_id_;           ///< name of the world frame id
  std::vector<Body *> excluded_bodies_;  ///< bodies to ignore
  /// whether to publish all collisions, or strictly adhere to update rate
  bool publish_all_collisions_;
  double update_rate_;  ///< rate to publish message at

  UpdateTimer update_timer_;  ///< for managing update rate

  /// For keeping track of contacts
  std::map<b2Contact *, ContactState> contact_states_;
  rclcpp::Publisher<flatland_msgs::msg::Collisions>::SharedPtr collisions_publisher_;  ///< For publishing the collisions

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

  /**
   * @brief Called when just after physics update
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

  /*
   * @brief A method that is called for Box2D presolve
   * @param[in] contact Box2D contact
   * @param[in] oldManifold Manifold from the previous iteration
   */
  void PostSolve(b2Contact *contact, const b2ContactImpulse *impulse) override;
};
};

#endif
