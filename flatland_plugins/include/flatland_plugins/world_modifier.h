// Copyright (c) 2017, Avidbots Corp.
// Copyright (c) 2026, Levent Soysal (Worthle).
// SPDX-License-Identifier: BSD-3-Clause
// Full license notices: LICENSE.

#ifndef WORLD_MODIFIER_H
#define WORLD_MODIFIER_H

#include <Box2D/Box2D.h>
#include <flatland_server/types.h>
#include <flatland_server/world.h>
#include <flatland_server/yaml_reader.h>
#include <yaml-cpp/yaml.h>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
using namespace flatland_server;

namespace flatland_plugins {

// sub class for loading laser Param
struct LaserRead {
  std::string name_;  // name of the laser
  b2Vec2 location_;   // location of the laser in local frame
  double range_;      // range of the laser
  double min_angle_;
  double max_angle_;
  double increment_;
  LaserRead(std::string name, b2Vec2 location, double range, double min_angle,
            double max_angle, double increment)
      : name_(name),
        location_(location),
        range_(range),
        min_angle_(min_angle),
        max_angle_(max_angle),
        increment_(increment) {}
};

struct RayTrace : public b2RayCastCallback {
  bool is_hit_;
  float fraction_;
  uint16_t category_bits_;
  RayTrace(uint16_t category_bits)
      : is_hit_(false), category_bits_(category_bits) {}
  float ReportFixture(b2Fixture *fixture, const b2Vec2 &point,
                      const b2Vec2 &normal, float fraction) override;
};

struct WorldModifier {
  // private members
  World *world_;  // the world we are modifying
  std::string layer_name_;
  double wall_wall_dist_;
  bool double_wall_;
  Pose robot_ini_pose_;

  /*
  * @brief based on the info regard old wall and d, calculate new obstacle's
  * vertices
  * @param[in] double d, the sign of this value determine which side of the wall
  *   will the new obstacle be added
  * @param[in] b2Vec2 vertex1, old wall's vertex1
  * @param[in] b2Vec2 vertex2, old wall's vertex2
  * @param[out] b2EdgeShape new_wall, reference passed in to set the vertices
  */
  void CalculateNewWall(double d, b2Vec2 vertex1, b2Vec2 vertex2,
                        b2EdgeShape &new_wall);

  /*
  * @brief add the new wall into the world
  * @param[in] new_wall, the wall that's going to be added
  */
  void AddWall(b2EdgeShape &new_wall);

  /*
  * @brief add two side walls to make it a full obstacle
  * @param[in] old_wall, the old wall where new wall is added on top to
  * @param[in] new_wall, the new wall got added
  */
  void AddSideWall(b2EdgeShape &old_wall, b2EdgeShape &new_wall);

  /*
   * @brief constructor for WorldModifier
   * @param[in] world, the world that we are adding walls to
   * @param[in] layer_name, which layer is the obstacle going to be added
   * @param[in] wall_wall_dist, how thick is the obstacle
   * @param[in] double_wall, whether add obstacle on both side or not
   * @param[in] robot_ini_pose, the initial pose of the robot
  */
  WorldModifier(flatland_server::World *world, std::string layer_name,
                double wall_wall_dist, bool double_wall, Pose robot_ini_pose);

  /*
  * @brief make a new wall in front of the old wall, also add two side walls to
  * make a full object
  * @param[in] b2EdgeShape *wall, old wall where new wall will be added on top
  * to
  */
  void AddFullWall(b2EdgeShape *wall);

};      // class WorldModifier
};      // namespace flatland_server
#endif  // WORLD_MODIFIER_H
