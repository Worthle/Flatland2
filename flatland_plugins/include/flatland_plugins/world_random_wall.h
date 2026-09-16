// Copyright (c) 2017, Avidbots Corp.
// Copyright (c) 2026, Levent Soysal (Worthle).
// SPDX-License-Identifier: BSD-3-Clause
// Full license notices: LICENSE.

#include <Box2D/Box2D.h>
#include <flatland_server/types.h>
#include <flatland_server/world_plugin.h>
#include <rclcpp/rclcpp.hpp>
#include <string>

#ifndef FLATLAND_PLUGINS_WORLD_RANDOM_WALL_H
#define FLATLAND_PLUGINS_WORLD_RANDOM_WALL_H

using namespace flatland_server;

namespace flatland_plugins {
class RandomWall : public WorldPlugin {
  void OnInitialize(const YAML::Node &config) override;
};
};

#endif  // FLATLAND_PLUGINS_WORLD_RANDOM_WALL_H
