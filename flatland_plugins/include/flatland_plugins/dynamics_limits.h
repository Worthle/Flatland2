// Copyright (c) 2021, Avidbots Corp.
// SPDX-License-Identifier: BSD-3-Clause
// Full license notices: LICENSE.

#ifndef FLATLAND_PLUGINS_DYNAMICS_LIMITS_H
#define FLATLAND_PLUGINS_DYNAMICS_LIMITS_H

#include <flatland_server/yaml_reader.h>
#include <yaml-cpp/yaml.h>

namespace flatland_plugins {

/**
 * This class implements the model plugin class and provides laser data
 * for the given configurations
 */
class DynamicsLimits {
 public:
  double acceleration_limit_ = 0.0;  // Maximum rate of change in velocity in the direction away from zero. Zero value disables this limit.
  double deceleration_limit_ = 0.0; // Maximum rate of change in velocity in the direction towards zero. Zero value disables this limit.
  double velocity_limit_ = 0.0;      // Maximum rate of change in position (in either direction). Zero value disables this limit.


  /**
   * @brief               blank constructor, no-op class
   */
  DynamicsLimits() {};

  /**
   * @name         Load configuration from a yaml object
   * @brief        Constructor from yaml configuration file node
   * @param[in]    YAML::Node& configuration node
   */
   void Configure(const YAML::Node &config);

  /**
   * @name          Saturate
   * @brief         Apply dynamics limits
   * @param[in]     config The plugin YAML node
   */
   static double Saturate(double in, double lower, double upper);

  /**
   * @name          Apply
   * @brief         Apply dynamics limits based on class configured limits
   * @param[in]     velocity The current velocity (units/second)
   * @param[in]     target_velocity The target velocity requested (units/second)
   * @param[in]     timestep  The timestep (seconds) (used to calculate acceleration from delta velocity)
   * @return        The new velocity result after target velocity has been subjected to limits
   */
   double Limit(double velocity, double target_velocity, double timestep);

};

};

#endif  // FLATLAND_PLUGINS_DYNAMICS_LIMITS_H
