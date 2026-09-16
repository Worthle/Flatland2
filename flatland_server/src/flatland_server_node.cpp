/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	flatland_server_node.cpp
 * @brief	Load params and run the ros node for flatland_server
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

#include <signal.h>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "flatland_server/debug_visualization.h"
#include "flatland_server/ros_node.h"
#include "flatland_server/simulation_manager.h"

/** Global variables */
flatland_server::SimulationManager *simulation_manager;

/**
 * @name        SigintHandler
 * @brief       Interrupt handler - sends shutdown signal to simulation_manager
 * @param[in]   sig: signal itself
 */
void SigintHandler(int sig) {
  RCLCPP_WARN(rclcpp::get_logger("Node"), "*** Shutting down... ***");

  if (simulation_manager != nullptr) {
    simulation_manager->Shutdown();
    delete simulation_manager;
    simulation_manager = nullptr;
  }
  RCLCPP_INFO(rclcpp::get_logger("Node"), "Beginning ros shutdown");
  rclcpp::shutdown();
}

/**
 * @name        main
 * @brief       Entrypoint for Flatland Server ros node
 */
int main(int argc, char **argv) {
  rclcpp::InitOptions init_options;
  init_options.shutdown_on_signal = false;
  rclcpp::init(argc, argv, init_options);

  // Create the single, global flatland node and store it for all flatland
  // classes to access (see ros_node.h). Allow undeclared params from overrides
  // so world.yaml Lua $(GetParam ...) and layer params resolve from launch.
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  node_options.allow_undeclared_parameters(true);
  flatland_server::ros_node() =
      std::make_shared<rclcpp::Node>("flatland", node_options);
  auto node = flatland_server::ros_node();

  // Load parameters. Parameters passed as overrides are already declared by
  // automatically_declare_parameters_from_overrides, so guard each declaration.
  auto declare_if_needed = [&](const std::string &name, auto default_value) {
    if (!node->has_parameter(name)) {
      node->declare_parameter(name, default_value);
    }
  };
  declare_if_needed("world_path", std::string(""));
  declare_if_needed("update_rate", 200.0);
  declare_if_needed("step_size", 1 / 200.0);
  declare_if_needed("show_viz", false);
  declare_if_needed("viz_pub_rate", 30.0);
  declare_if_needed("default_extrude_height", 0.0);

  std::string world_path = node->get_parameter("world_path").as_string();
  if (world_path.empty()) {
    RCLCPP_FATAL(node->get_logger(), "No world_path parameter given!");
    rclcpp::shutdown();
    return 1;
  }

  double update_rate = node->get_parameter("update_rate").as_double();
  double step_size = node->get_parameter("step_size").as_double();
  bool show_viz = node->get_parameter("show_viz").as_bool();
  double viz_pub_rate = node->get_parameter("viz_pub_rate").as_double();
  double default_extrude_height =
      node->get_parameter("default_extrude_height").as_double();

  // Apply the global default 3D extrusion height to the visualizer so all
  // polygon bodies without their own "extrude" render as 3D boxes.
  flatland_server::DebugVisualization::Get().default_extrude_height_ =
      default_extrude_height;

  // Create simulation manager object
  simulation_manager = new flatland_server::SimulationManager(
      world_path, update_rate, step_size, show_viz, viz_pub_rate);

  // Register sigint shutdown handler
  signal(SIGINT, SigintHandler);

  RCLCPP_INFO(node->get_logger(), "Initialized");
  simulation_manager->Main();

  RCLCPP_INFO(node->get_logger(), "Returned from simulation manager main");
  delete simulation_manager;
  simulation_manager = nullptr;
  rclcpp::shutdown();
  return 0;
}
