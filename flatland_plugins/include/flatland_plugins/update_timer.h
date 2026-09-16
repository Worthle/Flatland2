// Copyright (c) 2017, Avidbots Corp.
// Copyright (c) 2026, Levent Soysal (Worthle).
// SPDX-License-Identifier: BSD-3-Clause
// Full license notices: LICENSE.

#ifndef FLATLAND_PLUGINS_UPDATE_TIMER_H
#define FLATLAND_PLUGINS_UPDATE_TIMER_H

#include <flatland_server/timekeeper.h>
#include <rclcpp/rclcpp.hpp>

namespace flatland_plugins {

class UpdateTimer {
 public:
  rclcpp::Duration period_{0, 0};                  ///< period of update
  rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};  ///< last update time

  /**
   * @brief Update timer constructor
   */
  UpdateTimer();

  /**
   * @brief Set the update rate
   * @param[in] rate Rate in Hz
   */
  void SetRate(double rate);

  /**
   * Call this method to check if an update is required to keep with the
   * set update rate
   * @param[in] timekeeper The object that manages time for the simulation,
   * update timers get the simulation as well as step size for calculation
   */
  bool CheckUpdate(const flatland_server::Timekeeper &timekeeper);
};
};

#endif
