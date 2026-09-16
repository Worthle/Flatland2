// Copyright (c) 2017, Avidbots Corp.
// Copyright (c) 2026, Levent Soysal (Worthle).
// SPDX-License-Identifier: BSD-3-Clause
// Full license notices: LICENSE.

#include <flatland_plugins/update_timer.h>
#include <cstdint>

namespace flatland_plugins {

UpdateTimer::UpdateTimer()
    : period_(0, 0), last_update_time_(0, 0, RCL_ROS_TIME) {}

void UpdateTimer::SetRate(double rate) {
  if (rate == 0.0)
    period_ = rclcpp::Duration(INT32_MAX, 0);  // 1000 hours is infinity right?
  else
    period_ = rclcpp::Duration::from_seconds(1.0 / rate);
}

bool UpdateTimer::CheckUpdate(const flatland_server::Timekeeper &timekeeper) {
  if (fabs(period_.seconds()) < 1e-5) {
    return true;
  }

  // Naive way of keeping the update rate, will produce update rate always
  // slightly below the desired rate
  /*
  if (now - last_update_time_ > period_) {
    last_update_time_ = now;
    return true;
  }
  return false;
  */

  // Method obtained from Hector Gazebo Plugins, works well when the step size
  // is stable and close to max step size.
  // hector_gazebo/hector_gazebo_plugins/include/hector_gazebo_plugins/update_timer.h
  double step = timekeeper.GetMaxStepSize();
  double fraction =
      fmod(timekeeper.GetSimTime().seconds() + (step / 2.0), period_.seconds());

  if ((fraction >= 0.0) && (fraction < step)) {
    last_update_time_ = timekeeper.GetSimTime();
    return true;
  }

  return false;
}
};
