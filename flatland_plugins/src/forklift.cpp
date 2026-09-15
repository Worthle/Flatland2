// Copyright (c) 2026, Levent Soysal (Worthle).
// SPDX-License-Identifier: BSD-3-Clause
// Full license notices: LICENSE and SOURCE_NOTICES.

#include <flatland_plugins/forklift.h>
#include <flatland_server/timekeeper.h>
#include <flatland_server/yaml_reader.h>
#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>

using namespace flatland_server;

namespace flatland_plugins {

// per-model instance registry so sibling plugins (ForkController) can command
// the forks in-process; init, step and destruction all run on the single
// world-update thread, so no locking is needed
static std::unordered_map<Model *, Forklift *> forklift_registry;

Forklift *Forklift::LookupForModel(Model *model) {
  auto it = forklift_registry.find(model);
  return it == forklift_registry.end() ? nullptr : it->second;
}

Forklift::~Forklift() {
  if (LookupForModel(GetModel()) == this) {
    forklift_registry.erase(GetModel());
  }
}

void Forklift::SetTargetElevation(double elevation_m) {
  target_elevation_ =
      std::max(0.0, std::min(elevation_m, lift_height_));
  lifted_ = target_elevation_ > 0.0;
  if (lift_speed_ <= 0.0) {
    current_elevation_ = target_elevation_;
    ApplyElevation(current_elevation_);
  }
  PublishState();
}

void Forklift::OnInitialize(const YAML::Node &config) {
  YamlReader reader(config);

  // Names of the fork bodies to control (e.g. left_knife, right_knife)
  std::vector<std::string> body_names =
      reader.GetList<std::string>("bodies", 1, -1);

  // Pseudo-3D / color parameters
  lift_height_ = reader.Get<double>("lift_height", 0.2);
  fork_thickness_ = reader.Get<double>("fork_thickness", 0.1);
  lowered_color_ = reader.GetColor("lowered_color", Color(1, 0, 0, 0.75));
  lifted_color_ = reader.GetColor("lifted_color", Color(0, 1, 0, 0.75));
  // vertical animation speed (m/s); <= 0 keeps the legacy instant behavior
  lift_speed_ = reader.Get<double>("lift_speed", 0.0);

  // ROS interface
  std::string service_name = reader.Get<std::string>("service", "fork/lift");
  std::string state_topic = reader.Get<std::string>("state_topic", "fork/state");
  std::string height_topic =
      reader.Get<std::string>("height_topic", "fork/height");
  bool initial_lifted = reader.Get<bool>("initial_lifted", false);
  update_rate_ = reader.Get<double>("update_rate",
                                    std::numeric_limits<double>::infinity());

  reader.EnsureAccessedAllKeys();

  // Resolve the fork bodies and configure them for extruded 3D rendering
  for (const std::string &name : body_names) {
    Body *body = GetModel()->GetBody(name);
    if (body == nullptr) {
      throw YAMLException("Body with name \"" + name + "\" does not exist");
    }
    body->extrude_height_ = fork_thickness_;
    fork_bodies_.push_back(body);
  }

  // State publisher and lift/lower service (namespaced to this model)
  state_pub_ = nh_->create_publisher<std_msgs::msg::Bool>(
      state_topic, 1);
  height_pub_ = nh_->create_publisher<std_msgs::msg::Float32>(
      height_topic, 1);
  lift_srv_ = nh_->create_service<flatland_msgs::srv::LiftFork>(
      service_name,
      std::bind(&Forklift::OnLiftRequest, this, std::placeholders::_1,
                std::placeholders::_2));

  update_timer_.SetRate(update_rate_);
  forklift_registry[GetModel()] = this;

  // Apply and publish the initial state (no animation on startup)
  lifted_ = initial_lifted;
  target_elevation_ = lifted_ ? lift_height_ : 0.0;
  current_elevation_ = target_elevation_;
  ApplyElevation(current_elevation_);
  PublishState();

  RCLCPP_INFO(rclcpp::get_logger("Forklift"),
              "Initialized forklift with %zu fork body(ies), service(%s) "
              "state_topic(%s) lift_height(%f) fork_thickness(%f) "
              "lift_speed(%f)",
              fork_bodies_.size(), service_name.c_str(), state_topic.c_str(),
              lift_height_, fork_thickness_, lift_speed_);
}

void Forklift::ApplyElevation(double elevation) {
  // fade the color with the travel progress so the state reads at a glance
  double p = lift_height_ > 0.0 ? elevation / lift_height_ : (lifted_ ? 1 : 0);
  p = std::max(0.0, std::min(1.0, p));
  Color color(lowered_color_.r + (lifted_color_.r - lowered_color_.r) * p,
              lowered_color_.g + (lifted_color_.g - lowered_color_.g) * p,
              lowered_color_.b + (lifted_color_.b - lowered_color_.b) * p,
              lowered_color_.a + (lifted_color_.a - lowered_color_.a) * p);
  for (Body *body : fork_bodies_) {
    body->SetColor(color);
    body->elevation_ = elevation;
  }
}

void Forklift::PublishState() {
  std_msgs::msg::Bool msg;
  msg.data = lifted_;
  state_pub_->publish(msg);

  std_msgs::msg::Float32 height;
  height.data = current_elevation_;
  height_pub_->publish(height);
}

void Forklift::OnLiftRequest(
    const std::shared_ptr<flatland_msgs::srv::LiftFork::Request> request,
    std::shared_ptr<flatland_msgs::srv::LiftFork::Response> response) {
  if (request->fraction < 0.0 || request->fraction > 1.0) {
    response->success = false;
    response->message = "fraction must be within [0, 1], got " +
                        std::to_string(request->fraction);
    return;
  }

  SetTargetElevation(request->fraction * lift_height_);

  response->success = true;
  response->message = "forks moving to " + std::to_string(target_elevation_) +
                      " m (fraction " + std::to_string(request->fraction) +
                      ")";
}

void Forklift::BeforePhysicsStep(const Timekeeper &timekeeper) {
  if (current_elevation_ == target_elevation_ || lift_speed_ <= 0.0) {
    return;
  }
  double max_step = lift_speed_ * timekeeper.GetStepSize();
  double error = target_elevation_ - current_elevation_;
  if (std::fabs(error) <= max_step) {
    current_elevation_ = target_elevation_;
  } else {
    current_elevation_ += (error > 0 ? max_step : -max_step);
  }
  ApplyElevation(current_elevation_);
}

void Forklift::AfterPhysicsStep(const Timekeeper &timekeeper) {
  if (!update_timer_.CheckUpdate(timekeeper)) {
    return;
  }
  PublishState();
}
}  // namespace flatland_plugins

PLUGINLIB_EXPORT_CLASS(flatland_plugins::Forklift, flatland_server::ModelPlugin)
