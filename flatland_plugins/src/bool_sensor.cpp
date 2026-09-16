// Copyright (c) 2017, Avidbots Corp.
// Copyright (c) 2026, Levent Soysal (Worthle).
// SPDX-License-Identifier: BSD-3-Clause
// Full license notices: LICENSE.

#include <flatland_plugins/bool_sensor.h>
#include <flatland_server/timekeeper.h>
#include <flatland_server/yaml_reader.h>
#include <pluginlib/class_list_macros.hpp>

using namespace flatland_server;

namespace flatland_plugins {

void BoolSensor::OnInitialize(const YAML::Node &config) {
  YamlReader reader(config);

  // defaults
  std::string topic_name = reader.Get<std::string>("topic", "bool_sensor");
  update_rate_ = reader.Get<double>("update_rate",
                                    std::numeric_limits<double>::infinity());

  // sensor defaults to the first model in the list
  if (GetModel()->bodies_.size() == 0) {
    throw YAMLException("You didn't provide any bodies for model" +
                        GetModel()->name_);
  }
  std::string body_name =
      reader.Get<std::string>("body", GetModel()->bodies_[0]->name_);

  reader.EnsureAccessedAllKeys();

  // Load the body pointer
  body_ = GetModel()->GetBody(body_name);
  if (body_ == nullptr) {
    throw YAMLException("Body with name \"" + body_name + "\" does not exist");
  }

  // Set the update timer
  update_timer_.SetRate(update_rate_);

  // Init publisher
  publisher_ =
      nh_->create_publisher<std_msgs::msg::Bool>(topic_name, 1);

  RCLCPP_DEBUG(rclcpp::get_logger("BoolSensor"),
                  "Initialized with params: topic(%s) body(%s) "
                  "update_rate(%f)",
                  topic_name.c_str(), body_name.c_str(), update_rate_);
}

void BoolSensor::AfterPhysicsStep(const Timekeeper &timekeeper) {
  // Publish the boolean timer at the desired update rate
  if (!update_timer_.CheckUpdate(timekeeper)) {
    return;
  }

  // This logic allows collisions that occur and resolve faster than the update
  // rate to result in at least one "true" publish
  std_msgs::msg::Bool msg;
  if (hit_something_) {  // We hit something since the last publish
    msg.data = true;
    hit_something_ = false;  // Reset for next time
  } else {                   // Publish current state as usual
    if (collisions_ > 0) {
      msg.data = true;
    } else {
      msg.data = false;
    }
  }
  publisher_->publish(msg);
}

void BoolSensor::BeginContact(b2Contact *contact) {
  if (!FilterContact(contact)) return;

  // Skip collisions with other fixtures on this body
  if (contact->GetFixtureA()->GetBody() == contact->GetFixtureB()->GetBody()) {
    return;
  }

  collisions_++;
  hit_something_ = true;
}

void BoolSensor::EndContact(b2Contact *contact) {
  if (!FilterContact(contact)) return;

  // Skip collisions with other fixtures on this body
  if (contact->GetFixtureA()->GetBody() == contact->GetFixtureB()->GetBody()) {
    return;
  }

  collisions_--;
}
}  // End flatland_plugins namespace

PLUGINLIB_EXPORT_CLASS(flatland_plugins::BoolSensor,
                       flatland_server::ModelPlugin)
