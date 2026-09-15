// Copyright (c) 2021, Avidbots Corp.
// SPDX-License-Identifier: BSD-3-Clause
// Full license notices: LICENSE and SOURCE_NOTICES.

#include "flatland_plugins/dynamics_limits.h"

namespace flatland_plugins {

void DynamicsLimits::Configure(const YAML::Node &config) {
  flatland_server::YamlReader reader(config);
  acceleration_limit_ = reader.Get<double>("acceleration_limit", 0.0);  // 0.0 default disables the limit
  deceleration_limit_ = reader.Get<double>("deceleration_limit", std::numeric_limits<double>::infinity());

  // if no acceleration limit is set, default it to equal the acceleration limit
  if (deceleration_limit_ == std::numeric_limits<double>::infinity()) {
    deceleration_limit_ = acceleration_limit_;
  }

  velocity_limit_ = reader.Get<double>("velocity_limit", 0.0);   // 0.0 default disables the limit
}

double DynamicsLimits::Saturate(double in, double lower, double upper) {
  if (lower > upper) {
    return in;
  }
  double out = in;
  out = std::max(out, lower);
  out = std::min(out, upper);
  return out;
}

double DynamicsLimits::Limit(double velocity, double target_velocity, double timestep) {

  // if enabled, apply velocity limit
  if (velocity_limit_ != 0.0) {
    // Saturating the command also saturates the steering velocity
    target_velocity = DynamicsLimits::Saturate(target_velocity, -velocity_limit_, velocity_limit_);
  }

  // if the target velocity magnitude is larger, and in the same direction, we're accelerating
  double acceleration_limit;
  if (target_velocity == 0) {  // we can only be decellerating
    acceleration_limit = deceleration_limit_;
  } else if (velocity == 0) {  // we can only be accelerating
    acceleration_limit = acceleration_limit_;
  } else if (velocity*target_velocity < 0) {  // velocities are in different directions, we must be decelerating at least initially
    if (deceleration_limit_ != 0.0) {
      double initial_deceleration = DynamicsLimits::Saturate((target_velocity - velocity) / timestep, -deceleration_limit_, deceleration_limit_);
      double new_velocity = velocity + initial_deceleration * timestep;
      if (new_velocity*velocity>0 && acceleration_limit_ != 0.0)  {  // no zero velocity crossing, we're only decelerating
        acceleration_limit = deceleration_limit_;
      } else {  // We decelerate through a zero velocity crossing, both limits apply proportionally
        double deceleration_time = fabs(velocity)/deceleration_limit_;
        if (acceleration_limit_ == 0) {  // odd corner case where there's a deceleration limit but no acceleration limit
          acceleration_limit = 0;  // effectively no limit
        } else {  // we have both acceleration and deceleration limits, which apply proportionally
          acceleration_limit = deceleration_limit_ * deceleration_time/timestep + acceleration_limit_ * (1-deceleration_time/timestep);
        }
      }
      
    } else {
      velocity = 0;  // override incoming velocity, since we have no deceleration limit
      acceleration_limit = acceleration_limit_;
    }
  } else if (fabs(velocity) < fabs(target_velocity))  {  // new velocity magnitude is larger than old, and in the same direction: accelerating
    acceleration_limit = acceleration_limit_;
  } else {  // new velocity magnitude is smaller or equal than old, in the same direction: decelerating, unless there's a zero crossing inside the timestep
    acceleration_limit = deceleration_limit_;
  }

  // compute accleration, and limit it if needed
  double acceleration = (target_velocity - velocity) / timestep;
  if (acceleration_limit != 0.0) {
    acceleration =
        DynamicsLimits::Saturate(acceleration, -acceleration_limit, acceleration_limit);
  }

  // (2) Update the new steering velocity
  return velocity + acceleration * timestep;
}

}  /* end namespace flatland_plugins */
