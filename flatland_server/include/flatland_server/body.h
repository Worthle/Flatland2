/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	 body.h
 * @brief	 Defines Body
 * @author   Chunshang Li
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

#ifndef FLATLAND_BODY_H
#define FLATLAND_BODY_H

#include <flatland_server/entity.h>
#include <flatland_server/types.h>
#include <flatland_server/yaml_reader.h>
#include <yaml-cpp/yaml.h>

namespace flatland_server {

/**
 * This class defines a body in the simulation. It wraps around the Box2D
 * physics body providing extra data and useful methods
 */
class Body {
 public:
  Entity *entity_;         ///< The entity the body belongs to
  std::string name_;       ///< name of the body, unique within a model
  b2Body *physics_body_;   ///< Box2D physics body
  Color color_;            ///< color, for visualization
  YAML::Node properties_;  ///< Properties document for plugins to use

  /// Vertical (Z) offset applied when visualizing this body, in meters. Lets
  /// plugins lift a body off the ground plane for pseudo-3D visualization (e.g.
  /// forklift forks). Defaults to 0, so existing bodies render flat as before.
  double elevation_ = 0.0;
  /// When > 0, the body's polygon footprints are drawn as a filled 3D prism of
  /// this height (in meters) instead of a flat outline. Defaults to 0 (flat).
  double extrude_height_ = 0.0;

  /// Render-only offset; does not change collision, sensor height or TF.
  double visual_z_offset_ = 0.0;
  /// Optional RViz mesh URI; footprints still define planar physics and sensing.
  std::string visual_mesh_;
  struct WheelVisual {
    double radius = 0.0;  ///< Positive radius enables the cylindrical visual.
    double width = 0.0;
    Vec2 center{0, 0};    ///< Axle center in the body's planar coordinates.
    double rotation = 0.0;
    bool initialized = false;
    b2Vec2 last_position{0, 0};
    double last_heading = 0.0;
  } wheel_visual_;

  /**
   * @brief constructor for body, takes in all the required parameters
   * @param[in] physics_world Box2D physics world
   * @param[in] entity Entity the body is tied to
   * @param[in] name Name for the body
   * @param[in] color Color in r, g, b, a
   * @param[in] pose Pose to place the body at
   * @param[in] body_type Box2D body type either dynamic, kinematic, or static
   * @param[in] properties per-object YAML properties for plugins to reference
   * @param[in] linear_damping Box2D body linear damping
   * @param[in] angular_damping Box2D body angular damping
   */
  Body(b2World *physics_world, Entity *entity, const std::string &name,
       const Color &color, const Pose &pose, b2BodyType body_type,
       const YAML::Node &properties, double linear_damping = 0,
       double angular_damping = 0);

  /**
   * @brief logs the debugging information for the body
   */
  void DebugOutput() const;

  /**
   * @return entity associated with the body
   */
  Entity *GetEntity();

  /**
   * @return name of the body
   */
  const std::string &GetName() const;

  /**
   * @brief Get the Box2D body, use this to manipulate the body in physics
   * through the Box2D methods
   * @return Pointer to Box2D physics body
   */
  b2Body *GetPhysicsBody();

  /**
   * @brief Count the number of fixtures
   * @return number of fixtures in the body
   */
  int GetFixturesCount() const;

  /**
   * @return Color of the body
   */
  const Color &GetColor() const;

  /**
   * @brief Set of the color of the body
   */
  void SetColor(const Color &color);

  /**
   * Destructor for the body
   */
  virtual ~Body();

  /**
   * Prevent copying since it is creates complications with constructing
   * and destructing bodies
   */
  Body(const Body &) = delete;
  Body &operator=(const Body &) = delete;
};
};      // namespace flatland_server
#endif  // FLATLAND_MODEL_BODY_H
