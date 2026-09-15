/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	simulation_manager.cpp
 * @brief	Simulation manager runs the physics+event loop
 * @author Joseph Duchesne
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Avidbots Corp.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *      copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Avidbots Corp. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include "flatland_server/simulation_manager.h"
#include <flatland_server/debug_visualization.h>
#include <flatland_server/layer.h>
#include <flatland_server/model.h>
#include <flatland_server/service_manager.h>
#include <flatland_server/world.h>
#include <flatland_server/ros_node.h>
#include <rclcpp/rclcpp.hpp>
#include <exception>
#include <limits>
#include <string>
#include <chrono>
#include <thread>

namespace flatland_server {

SimulationManager::SimulationManager(std::string world_yaml_file,
                                     double update_rate, double step_size,
                                     bool show_viz, double viz_pub_rate)
    : world_(nullptr),
      update_rate_(update_rate),
      step_size_(step_size),
      show_viz_(show_viz),
      viz_pub_rate_(viz_pub_rate),
      world_yaml_file_(world_yaml_file) {
  RCLCPP_INFO(rclcpp::get_logger("SimMan"),
              "Simulation params: world_yaml_file(%s) update_rate(%f), "
              "step_size(%f) show_viz(%s), viz_pub_rate(%f)",
              world_yaml_file_.c_str(), update_rate_, step_size_,
              show_viz_ ? "true" : "false", viz_pub_rate_);
}

void SimulationManager::Main(bool benchmark) {
  RCLCPP_INFO(rclcpp::get_logger("SimMan"), "Initializing...");
  run_simulator_ = true;

  try {
    world_ = World::MakeWorld(world_yaml_file_);
    RCLCPP_INFO(rclcpp::get_logger("SimMan"), "World loaded");
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("SimMan"), "%s", e.what());
    return;
  }

  if (show_viz_) world_->DebugVisualize();

  iterations_ = 0;
  double filtered_cycle_util = 0;
  double min_cycle_util = std::numeric_limits<double>::infinity();
  double max_cycle_util = 0;
  double viz_update_period = 1.0f / viz_pub_rate_;
  ServiceManager service_manager(this, world_);

  // Track cycle utilization and compensate the next sleep for elapsed work.
  std::chrono::duration<double> start = std::chrono::steady_clock::now().time_since_epoch();
  std::chrono::duration<double> expected_cycle_time(1.0/update_rate_);
  std::chrono::duration<double> actual_cycle_time(0.0);
  using seconds_d = std::chrono::duration<double, std::ratio<1, 1>>;
  double seconds_taken = 0;

  timekeeper_.SetMaxStepSize(step_size_);
  RCLCPP_INFO(rclcpp::get_logger("SimMan"), "Simulation loop started");

  while (rclcpp::ok() && run_simulator_) {
    // for updating visualization at a given rate
    // see flatland_plugins/update_timer.cpp for this formula
    double f = 0.0;
    try {
      f = fmod(timekeeper_.GetSimTime().seconds() +
                   (expected_cycle_time.count() / 2.0),
               viz_update_period);
    } catch (std::runtime_error& ex) {
      RCLCPP_ERROR(rclcpp::get_logger("SimMan"),
                   "Flatland runtime error: [%s]", ex.what());
    }
    std::chrono::duration<double> update_start = std::chrono::steady_clock::now().time_since_epoch();
    bool update_viz = ((f >= 0.0) && (f < expected_cycle_time.count()));

    world_->Update(timekeeper_);  // Step physics by ros cycle time

    if (show_viz_ && update_viz) {
      world_->DebugVisualize(false);  // no need to update layer
      DebugVisualization::Get().Publish(
          timekeeper_);  // publish debug visualization
    }

    rclcpp::spin_some(ros_node());

    seconds_taken += (seconds_d(std::chrono::steady_clock::now().time_since_epoch()) - update_start).count();

    // Use monotonic wall time to pace simulation steps.
    {
      std::chrono::duration<double> expected_end = start + expected_cycle_time;
      std::chrono::duration<double> actual_end = std::chrono::steady_clock::now().time_since_epoch();
      std::chrono::duration<double> sleep_time = expected_end - actual_end;  //calculate the time we'll sleep for
      actual_cycle_time = actual_end - start;
      start = expected_end;  //make sure to reset our start time
      if(sleep_time.count() <= 0.0) { //if we've taken too much time we won't sleep
        if (actual_end > expected_end + expected_cycle_time) {
          start = actual_end;
        }
      } else {  // sleep, unless we're in a benchmark
        if (benchmark == false) {   // if benchmark==true, skip sleeping to run as fast as possible
          std::this_thread::sleep_for(sleep_time);
        } else {
          start = actual_end;
        }
      }
    }

    iterations_++;

    double cycle_time = actual_cycle_time.count() * 1000;
    double expected_cycle_time_ms = expected_cycle_time.count() * 1000;
    double cycle_util = cycle_time / expected_cycle_time_ms * 100;  // in percent
    double factor = timekeeper_.GetStepSize() * 1000 / expected_cycle_time_ms;
    min_cycle_util = std::min(cycle_util, min_cycle_util);
    if (iterations_ > 10) max_cycle_util = std::max(cycle_util, max_cycle_util);
    filtered_cycle_util = 0.99 * filtered_cycle_util + 0.01 * cycle_util;

    RCLCPP_INFO_THROTTLE(
        rclcpp::get_logger("SimMan"), *ros_node()->get_clock(), 1000,
        "utilization: min %.1f%% max %.1f%% ave %.1f%%  factor: %.1f",
        min_cycle_util, max_cycle_util, filtered_cycle_util, factor);
  }
  // std::cout << "Simulation loop ended. " << iterations_ << " iterations in " << seconds_taken << " seconds, " <<  (double)iterations_/seconds_taken << " iterations/sec" << std::endl;

  delete world_;
}

void SimulationManager::Shutdown() {
  RCLCPP_INFO(rclcpp::get_logger("SimMan"), "Shutdown called");
  run_simulator_ = false;
}

};  // namespace flatland_server
