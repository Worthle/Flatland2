// Copyright (c) 2017, Avidbots Corp.
// Copyright (c) 2026, Levent Soysal (Worthle).
// SPDX-License-Identifier: BSD-3-Clause
// Full license notices: LICENSE and SOURCE_NOTICES.

#include <Box2D/Box2D.h>
#include <flatland_plugins/update_timer.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/timekeeper.h>
#include <flatland_server/types.h>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include "tweeny.h"

#ifndef FLATLAND_PLUGINS_TWEEN_H
#define FLATLAND_PLUGINS_TWEEN_H

using namespace flatland_server;

namespace flatland_plugins {

class Tween : public flatland_server::ModelPlugin {
 public:
  Body* body_;      // The body this plugin is attached to
  Pose start_;      // The start pose of the model
  Pose delta_;      // The maximum change
  float duration_;  // Seconds to enact change over

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr trigger_sub_;  // Handle forward/reverse trigger
  bool triggered_ = false;  // If true,animate forwards, otherwise backwards

  tweeny::tween<double, double, double> tween_;  // The tween object (x,y,theta)

  // The three different operating modes
  enum class ModeType_ {
    YOYO,    // tween up to delta_, then down again, and repeat
    LOOP,    // tween up to delta_, then teleport back to start_
    ONCE,    // tween up to delta_ then stay there
    TRIGGER  // tween forwards on "true", backward on "false"
  };
  ModeType_ mode_;
  static std::map<std::string, Tween::ModeType_> mode_strings_;

  enum class EasingType_ {
    linear,
    quadraticIn,
    quadraticOut,
    quadraticInOut,
    cubicIn,
    cubicOut,
    cubicInOut,
    quarticIn,
    quarticOut,
    quarticInOut,
    quinticIn,
    quinticOut,
    quinticInOut,
    // sinuisodal,
    exponentialIn,
    exponentialOut,
    exponentialInOut,
    circularIn,
    circularOut,
    circularInOut,
    backIn,
    backOut,
    backInOut,
    elasticIn,
    elasticOut,
    elasticInOut,
    bounceIn,
    bounceOut,
    bounceInOut
  };
  static std::map<std::string, Tween::EasingType_> easing_strings_;

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
   * @name      TriggerCallback
   * @brief     Handles external tween triggers for mode "trigger"
   * @param[in] The boolean message
   */
  void TriggerCallback(const std_msgs::msg::Bool& msg);
};
};

#endif
