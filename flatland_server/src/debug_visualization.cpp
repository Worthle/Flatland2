/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name   debug_visualization.cpp
 * @brief  Transform box2d types into published visualization messages
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

#include "flatland_server/debug_visualization.h"
#include <Box2D/Box2D.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace flatland_server {

DebugVisualization::DebugVisualization() : node_(ros_node()) {
  // ROS 1 "latched" publisher -> ROS 2 transient_local durability.
  rclcpp::QoS latched_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
  topic_list_publisher_ =
      node_->create_publisher<flatland_msgs::msg::DebugTopicList>(
          "/flatland_server/debug/topics", latched_qos);
}

DebugVisualization& DebugVisualization::Get() {
  static DebugVisualization instance;
  return instance;
}

void DebugVisualization::JointToMarkers(
    visualization_msgs::msg::MarkerArray& markers, b2Joint* joint, float r,
    float g, float b, float a) {
  if (joint->GetType() == e_distanceJoint ||
      joint->GetType() == e_pulleyJoint || joint->GetType() == e_mouseJoint) {
    RCLCPP_ERROR(rclcpp::get_logger("DebugVis"),
                 "Unimplemented visualization joints. See b2World.cpp for "
                 "implementation");
    return;
  }

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = a;
  marker.type = marker.LINE_LIST;
  marker.scale.x = 0.01;

  geometry_msgs::msg::Point p_a1, p_a2, p_b1, p_b2;
  p_a1.x = joint->GetAnchorA().x;
  p_a1.y = joint->GetAnchorA().y;
  p_a2.x = joint->GetAnchorB().x;
  p_a2.y = joint->GetAnchorB().y;
  p_b1.x = joint->GetBodyA()->GetPosition().x;
  p_b1.y = joint->GetBodyA()->GetPosition().y;
  p_b2.x = joint->GetBodyB()->GetPosition().x;
  p_b2.y = joint->GetBodyB()->GetPosition().y;

  // Visualization shows lines from bodyA to anchorA, bodyB to anchorB, and
  // anchorA to anchorB
  marker.id = markers.markers.size();
  marker.points.push_back(p_b1);
  marker.points.push_back(p_a1);
  marker.points.push_back(p_b2);
  marker.points.push_back(p_a2);
  marker.points.push_back(p_a1);
  marker.points.push_back(p_a2);

  markers.markers.push_back(marker);

  marker.id = markers.markers.size();
  marker.type = marker.CUBE_LIST;
  marker.scale.x = marker.scale.y = marker.scale.z = 0.03;
  marker.points.clear();
  marker.points.push_back(p_a1);
  marker.points.push_back(p_a2);
  marker.points.push_back(p_b1);
  marker.points.push_back(p_b2);
  markers.markers.push_back(marker);
}

namespace {
void WheelToMarkers(visualization_msgs::msg::MarkerArray& markers, Body& body) {
  using Marker = visualization_msgs::msg::Marker;
  auto& wheel = body.wheel_visual_;
  auto* physics = body.physics_body_;
  const double radius = wheel.radius, width = wheel.width;
  const b2Vec2 center =
      physics->GetWorldPoint(b2Vec2(wheel.center.x, wheel.center.y));
  const double heading = physics->GetAngle();
  if (wheel.initialized) {
    const double middle =
        wheel.last_heading +
        0.5 * std::remainder(heading - wheel.last_heading, 2.0 * M_PI);
    const b2Vec2 travel = center - wheel.last_position;
    wheel.rotation = std::remainder(
        wheel.rotation +
            (travel.x * std::cos(middle) + travel.y * std::sin(middle)) /
                radius,
        2.0 * M_PI);
  }
  wheel.last_position = center;
  wheel.last_heading = heading;
  wheel.initialized = true;

  tf2::Quaternion axle, roll;
  axle.setRPY(-M_PI / 2.0, 0,
              heading);  // Cylinder Z axis becomes the wheel axle (+Y).
  roll.setRPY(0, 0, wheel.rotation);
  Marker base;
  base.header.frame_id = "map";
  base.pose.position.x = center.x;
  base.pose.position.y = center.y;
  base.pose.position.z = body.elevation_ + body.visual_z_offset_ + radius;
  base.pose.orientation = tf2::toMsg(axle * roll);
  base.color.a = body.color_.a;

  auto add = [&](Marker marker, const std::string& part) {
    marker.id = markers.markers.size();
    marker.ns = "wheel/" + body.name_ + "/" + part;
    markers.markers.push_back(std::move(marker));
  };
  auto cylinder = [&](const std::string& part, double r, double w, float red,
                      float green, float blue) {
    Marker marker = base;
    marker.type = Marker::CYLINDER;
    marker.scale.x = marker.scale.y = 2.0 * r;
    marker.scale.z = w;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    add(marker, part);
  };
  cylinder("tire", radius, width, 0.055f, 0.065f, 0.075f);
  cylinder("rim", radius * 0.72, width * 1.04, 0.58f, 0.64f, 0.70f);
  cylinder("recess", radius * 0.60, width * 1.06, 0.12f, 0.16f, 0.20f);
  cylinder("hub", radius * 0.20, width * 1.15, 0.76f, 0.80f, 0.84f);

  auto point = [](double x, double y, double z) {
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
  };
  Marker spokes = base;
  spokes.type = Marker::LINE_LIST;
  spokes.scale.x = radius * 0.065;
  spokes.color.r = 0.70f;
  spokes.color.g = 0.75f;
  spokes.color.b = 0.80f;
  for (double side : {-1.0, 1.0}) {
    for (int i = 0; i < 6; ++i) {
      const double angle = i * M_PI / 3.0;
      for (double r : {radius * 0.22, radius * 0.58}) {
        spokes.points.push_back(point(r * std::cos(angle), r * std::sin(angle),
                                      side * width * 0.54));
      }
    }
  }
  add(spokes, "spokes");

  Marker tread = base;
  tread.type = Marker::LINE_LIST;
  tread.scale.x = radius * 0.025;
  tread.color.r = 0.13f;
  tread.color.g = 0.15f;
  tread.color.b = 0.17f;
  for (int i = 0; i < 16; ++i) {
    const double angle = i * M_PI / 8.0;
    const double x = radius * 1.005 * std::cos(angle);
    const double y = radius * 1.005 * std::sin(angle);
    tread.points.push_back(point(x, y, -width * 0.43));
    tread.points.push_back(point(x, y, width * 0.43));
  }
  add(tread, "tread");

  // A trailing caster also gets a stationary fork between its pivot and axle.
  if (std::hypot(wheel.center.x, wheel.center.y) > 1e-6) {
    Marker bracket = base;
    bracket.type = Marker::LINE_LIST;
    bracket.scale.x = radius * 0.13;
    bracket.color.r = body.color_.r;
    bracket.color.g = body.color_.g;
    bracket.color.b = body.color_.b;
    bracket.pose.position.x = physics->GetPosition().x;
    bracket.pose.position.y = physics->GetPosition().y;
    bracket.pose.position.z = body.elevation_ + body.visual_z_offset_;
    tf2::Quaternion yaw;
    yaw.setRPY(0, 0, heading);
    bracket.pose.orientation = tf2::toMsg(yaw);
    for (double side : {-1.0, 1.0}) {
      bracket.points.push_back(point(0, side * width * 0.68, radius * 2.15));
      bracket.points.push_back(
          point(wheel.center.x, wheel.center.y + side * width * 0.68, radius));
    }
    bracket.points.push_back(point(0, -width * 0.68, radius * 2.15));
    bracket.points.push_back(point(0, width * 0.68, radius * 2.15));
    add(bracket, "caster_fork");
  }
}
}  // namespace

void DebugVisualization::BodyToMarkers(
    visualization_msgs::msg::MarkerArray& markers, b2Body* body, float r,
    float g, float b, float a) {
  b2Fixture* fixture = body->GetFixtureList();

  // The flatland Body (if any) is stored as the Box2D body user data. It
  // carries optional pseudo-3D render parameters (elevation, extrusion) that
  // plugins can update at runtime, e.g. to raise forklift forks.
  Body* fl_body = static_cast<Body*>(body->GetUserData());
  if (fl_body && !fl_body->visual_mesh_.empty()) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.ns = "mesh/" + fl_body->name_;
    marker.id = markers.markers.size();
    marker.type = marker.MESH_RESOURCE;
    marker.mesh_resource = fl_body->visual_mesh_;
    marker.mesh_use_embedded_materials = true;
    // Zero RGBA preserves the mesh's original materials without RViz tinting.
    marker.scale.x = marker.scale.y = marker.scale.z = 1.0;
    marker.pose.position.x = body->GetPosition().x;
    marker.pose.position.y = body->GetPosition().y;
    marker.pose.position.z = fl_body->elevation_ + fl_body->visual_z_offset_;
    tf2::Quaternion yaw;
    yaw.setRPY(0, 0, body->GetAngle());
    marker.pose.orientation = tf2::toMsg(yaw);
    markers.markers.push_back(marker);
    return;
  }
  if (fl_body && fl_body->wheel_visual_.radius > 0.0) {
    WheelToMarkers(markers, *fl_body);
    return;
  }
  double elevation =
      fl_body ? fl_body->elevation_ + fl_body->visual_z_offset_ : 0.0;
  double extrude_height = fl_body ? fl_body->extrude_height_ : 0.0;
  // Fall back to the global default extrusion for bodies that don't set their
  // own, so all polygon bodies render as 3D boxes when a default is configured.
  if (extrude_height <= 0.0) extrude_height = default_extrude_height_;

  while (fixture != NULL) {  // traverse fixture linked list
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.id = markers.markers.size();
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
    marker.pose.position.x = body->GetPosition().x;
    marker.pose.position.y = body->GetPosition().y;
    marker.pose.position.z = elevation;
    tf2::Quaternion q;  // use tf2 to convert 2d yaw -> 3d quaternion
    q.setRPY(0, 0, body->GetAngle());  // from euler angles: roll, pitch, yaw
    marker.pose.orientation = tf2::toMsg(q);
    bool add_marker = true;

    // When the body requests extrusion and the fixture is a polygon, render a
    // filled 3D prism (z in [0, extrude_height], offset by the marker pose's
    // elevation) instead of a flat outline.
    if (extrude_height > 0.0 && fixture->GetType() == b2Shape::e_polygon) {
      b2PolygonShape* poly = (b2PolygonShape*)fixture->GetShape();
      marker.type = marker.TRIANGLE_LIST;
      marker.scale.x = marker.scale.y = marker.scale.z = 1.0;

      auto pt = [](float x, float y, float z) {
        geometry_msgs::msg::Point p;
        p.x = x;
        p.y = y;
        p.z = z;
        return p;
      };

      int n = poly->m_count;
      // Bottom and top faces (triangle fans from vertex 0)
      for (int i = 1; i < n - 1; i++) {
        // bottom (z = 0)
        marker.points.push_back(
            pt(poly->m_vertices[0].x, poly->m_vertices[0].y, 0));
        marker.points.push_back(
            pt(poly->m_vertices[i].x, poly->m_vertices[i].y, 0));
        marker.points.push_back(
            pt(poly->m_vertices[i + 1].x, poly->m_vertices[i + 1].y, 0));
        // top (z = extrude_height)
        marker.points.push_back(
            pt(poly->m_vertices[0].x, poly->m_vertices[0].y, extrude_height));
        marker.points.push_back(pt(poly->m_vertices[i + 1].x,
                                   poly->m_vertices[i + 1].y, extrude_height));
        marker.points.push_back(
            pt(poly->m_vertices[i].x, poly->m_vertices[i].y, extrude_height));
      }
      // Side walls (two triangles per edge)
      for (int i = 0; i < n; i++) {
        int j = (i + 1) % n;
        b2Vec2 vi = poly->m_vertices[i];
        b2Vec2 vj = poly->m_vertices[j];
        marker.points.push_back(pt(vi.x, vi.y, 0));
        marker.points.push_back(pt(vj.x, vj.y, 0));
        marker.points.push_back(pt(vj.x, vj.y, extrude_height));
        marker.points.push_back(pt(vi.x, vi.y, 0));
        marker.points.push_back(pt(vj.x, vj.y, extrude_height));
        marker.points.push_back(pt(vi.x, vi.y, extrude_height));
      }

      markers.markers.push_back(marker);
      fixture = fixture->GetNext();
      continue;
    }

    // Get the shape from the fixture
    switch (fixture->GetType()) {
      case b2Shape::e_circle: {
        b2CircleShape* circle = (b2CircleShape*)fixture->GetShape();

        marker.type = marker.SPHERE_LIST;
        float diameter = circle->m_radius * 2.0;
        marker.scale.z = 0.01;
        marker.scale.x = diameter;
        marker.scale.y = diameter;

        geometry_msgs::msg::Point p;
        p.x = circle->m_p.x;
        p.y = circle->m_p.y;
        marker.points.push_back(p);

      } break;

      case b2Shape::e_polygon: {  // Convert b2Polygon -> LINE_STRIP
        b2PolygonShape* poly = (b2PolygonShape*)fixture->GetShape();
        marker.type = marker.LINE_STRIP;
        marker.scale.x = 0.03;  // 3cm wide lines

        for (int i = 0; i < poly->m_count; i++) {
          geometry_msgs::msg::Point p;
          p.x = poly->m_vertices[i].x;
          p.y = poly->m_vertices[i].y;
          marker.points.push_back(p);
        }
        marker.points.push_back(marker.points[0]);  // Close the shape

      } break;

      case b2Shape::e_edge: {         // Convert b2Edge -> LINE_LIST
        geometry_msgs::msg::Point p;  // b2Edge uses vertex1 and 2 for its edges
        b2EdgeShape* edge = (b2EdgeShape*)fixture->GetShape();

        // If the last marker is a line list, extend it
        if (markers.markers.size() > 0 &&
            markers.markers.back().type == marker.LINE_LIST) {
          add_marker = false;
          p.x = edge->m_vertex1.x;
          p.y = edge->m_vertex1.y;
          markers.markers.back().points.push_back(p);
          p.x = edge->m_vertex2.x;
          p.y = edge->m_vertex2.y;
          markers.markers.back().points.push_back(p);

        } else {  // otherwise create a new line list

          marker.type = marker.LINE_LIST;
          marker.scale.x = 0.03;  // 3cm wide lines

          p.x = edge->m_vertex1.x;
          p.y = edge->m_vertex1.y;
          marker.points.push_back(p);
          p.x = edge->m_vertex2.x;
          p.y = edge->m_vertex2.y;
          marker.points.push_back(p);
        }

      } break;

      case b2Shape::e_chain: {
        geometry_msgs::msg::Point p;  // b2Edge uses vertex1 and 2 for its edges
        b2ChainShape* chain = (b2ChainShape*)fixture->GetShape();

        add_marker = true;
        marker.type = marker.LINE_STRIP;
        marker.scale.x = 0.03;  // 3cm wide lines

        for (int i = 0; i < chain->m_count; i++) {
          p.x = chain->m_vertices[i].x;
          p.y = chain->m_vertices[i].y;
          marker.points.push_back(p);
        }

        // close loop
        p.x = chain->m_vertices[0].x;
        p.y = chain->m_vertices[0].y;
        marker.points.push_back(p);
      } break;

      default:  // Unsupported shape
        RCLCPP_WARN_THROTTLE(
            rclcpp::get_logger("DebugVis"), *ros_node()->get_clock(), 1000,
            "Unsupported Box2D shape %d", static_cast<int>(fixture->GetType()));
        fixture = fixture->GetNext();
        continue;  // Do not add broken marker
        break;
    }

    if (add_marker) {
      markers.markers.push_back(marker);  // Add the new marker
    }
    fixture = fixture->GetNext();  // Traverse the linked list of fixtures
  }
}

void DebugVisualization::Publish(const Timekeeper& timekeeper) {
  // Iterate over the topics_ map as pair(name, topic)

  std::vector<std::string> to_delete;

  for (auto& topic : topics_) {
    if (!topic.second.needs_publishing) {
      continue;
    }

    // since if empty markers are published rviz will continue to publish
    // using the old data, delete the topic list
    if (topic.second.markers.markers.size() == 0) {
      to_delete.push_back(topic.first);
    } else {
      // Iterate the marker array to update all the timestamps
      for (unsigned int i = 0; i < topic.second.markers.markers.size(); i++) {
        topic.second.markers.markers[i].header.stamp = timekeeper.GetSimTime();
      }
      topic.second.publisher->publish(topic.second.markers);
      topic.second.needs_publishing = false;
    }
  }

  if (to_delete.size() > 0) {
    for (const auto& topic : to_delete) {
      RCLCPP_WARN(rclcpp::get_logger("DebugVis"), "Deleting topic %s",
                  topic.c_str());
      topics_.erase(topic);
    }
    PublishTopicList();
  }
}

void DebugVisualization::VisualizeLayer(std::string name, Body* body) {
  AddTopicIfNotExist(name);

  b2Fixture* fixture = body->physics_body_->GetFixtureList();

  visualization_msgs::msg::Marker marker;
  if (fixture == NULL) return;  // Nothing to visualize, empty linked list

  while (fixture != NULL) {  // traverse fixture linked list

    marker.header.frame_id = "map";
    marker.id = topics_[name].markers.markers.size();
    marker.color.r = body->color_.r;
    marker.color.g = body->color_.g;
    marker.color.b = body->color_.b;
    marker.color.a = body->color_.a;
    marker.scale.x = marker.scale.y = marker.scale.z = 1.0;
    marker.frame_locked = true;
    marker.pose.position.x = body->physics_body_->GetPosition().x;
    marker.pose.position.y = body->physics_body_->GetPosition().y;

    tf2::Quaternion q;  // use tf2 to convert 2d yaw -> 3d quaternion
    q.setRPY(0, 0, body->physics_body_
                       ->GetAngle());  // from euler angles: roll, pitch, yaw
    marker.pose.orientation = tf2::toMsg(q);
    marker.type = marker.TRIANGLE_LIST;

    YamlReader reader(body->properties_);
    YamlReader debug_reader =
        reader.SubnodeOpt("debug", YamlReader::NodeTypeCheck::MAP);
    float min_z = debug_reader.Get<float>("min_z", 0.0);
    float max_z = debug_reader.Get<float>("max_z", 1.0);

    // Get the shape from the fixture
    if (fixture->GetType() == b2Shape::e_edge) {
      geometry_msgs::msg::Point p;  // b2Edge uses vertex1 and 2 for its edges
      b2EdgeShape* edge = (b2EdgeShape*)fixture->GetShape();

      p.x = edge->m_vertex1.x;
      p.y = edge->m_vertex1.y;
      p.z = min_z;
      marker.points.push_back(p);
      p.x = edge->m_vertex2.x;
      p.y = edge->m_vertex2.y;
      p.z = min_z;
      marker.points.push_back(p);
      p.x = edge->m_vertex2.x;
      p.y = edge->m_vertex2.y;
      p.z = max_z;
      marker.points.push_back(p);

      p.x = edge->m_vertex1.x;
      p.y = edge->m_vertex1.y;
      p.z = min_z;
      marker.points.push_back(p);
      p.x = edge->m_vertex2.x;
      p.y = edge->m_vertex2.y;
      p.z = max_z;
      marker.points.push_back(p);
      p.x = edge->m_vertex1.x;
      p.y = edge->m_vertex1.y;
      p.z = max_z;
      marker.points.push_back(p);
    }

    fixture = fixture->GetNext();  // Traverse the linked list of fixtures
  }

  topics_[name].markers.markers.push_back(marker);  // Add the new marker
  topics_[name].needs_publishing = true;
}

void DebugVisualization::Visualize(std::string name, b2Body* body, float r,
                                   float g, float b, float a) {
  AddTopicIfNotExist(name);
  BodyToMarkers(topics_[name].markers, body, r, g, b, a);
  topics_[name].needs_publishing = true;
}

void DebugVisualization::Visualize(std::string name, b2Joint* joint, float r,
                                   float g, float b, float a) {
  AddTopicIfNotExist(name);
  JointToMarkers(topics_[name].markers, joint, r, g, b, a);
  topics_[name].needs_publishing = true;
}

void DebugVisualization::Reset(std::string name) {
  if (topics_.count(name) > 0) {  // If the topic exists, clear it
    topics_[name].markers.markers.clear();
    topics_[name].needs_publishing = true;
  }
}

void DebugVisualization::AddTopicIfNotExist(const std::string& name) {
  // If the topic doesn't exist yet, create it
  if (topics_.count(name) == 0) {
    rclcpp::QoS latched_qos =
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
    topics_[name] = {
        node_->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/flatland_server/debug/" + name, latched_qos),
        true, visualization_msgs::msg::MarkerArray()};

    RCLCPP_INFO_ONCE(rclcpp::get_logger("DebugVis"), "Visualizing %s",
                     name.c_str());
    PublishTopicList();
  }
}

void DebugVisualization::PublishTopicList() {
  flatland_msgs::msg::DebugTopicList topic_list;
  for (auto const& topic_pair : topics_)
    topic_list.topics.push_back(topic_pair.first);
  topic_list_publisher_->publish(topic_list);
}
};  // namespace flatland_server
