// Copyright (c) 2026, Levent Soysal (Worthle).
// SPDX-License-Identifier: BSD-3-Clause
// Full license notices: LICENSE and SOURCE_NOTICES.

#include <flatland_plugins/fork_controller.h>
#include <flatland_plugins/forklift.h>
#include <flatland_plugins/link_attacher.h>
#include <flatland_server/timekeeper.h>
#include <flatland_server/yaml_reader.h>
#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

using namespace flatland_server;

namespace flatland_plugins {

void ForkController::OnInitialize(const YAML::Node &config) {
  YamlReader reader(config);

  // Relative topic names inherit the robot namespace.
  std::string goal_topic =
      reader.Get<std::string>("fork_goal_topic", "fork/goal_height");
  std::string pose_topic =
      reader.Get<std::string>("fork_pose_topic", "fork/pose");
  std::string completed_topic = reader.Get<std::string>(
      "fork_goal_completed_topic", "fork/goal_reached");
  std::string weight_topic =
      reader.Get<std::string>("weight_topic", "fork/weight");
  std::string contact_left_topic =
      reader.Get<std::string>("contact_left_topic", "fork/contact_left");
  std::string contact_right_topic =
      reader.Get<std::string>("contact_right_topic", "fork/contact_right");
  std::string collision_left_topic =
      reader.Get<std::string>("collision_left_topic", "fork/collision_left");
  std::string collision_right_topic =
      reader.Get<std::string>("collision_right_topic", "fork/collision_right");
  std::string load_state_topic =
      reader.Get<std::string>("load_state_topic", "fork/loaded");
  pose_frame_ = GetModel()->NameSpaceTF(reader.Get<std::string>("pose_frame", "base_link"));

  goal_tolerance_ = reader.Get<double>("goal_tolerance", 0.02);
  contact_range_ = reader.Get<double>("contact_range", 0.2);
  loaded_weight_ = reader.Get<double>("loaded_weight", 300.0);
  auto_attach_ = reader.Get<bool>("auto_attach", true);
  attach_height_ = reader.Get<double>("attach_height", 0.05);
  detach_height_ = reader.Get<double>("detach_height", 0.02);
  update_rate_ = reader.Get<double>("update_rate", 20.0);

  reader.EnsureAccessedAllKeys();

  goal_sub_ = nh_->create_subscription<std_msgs::msg::Float32>(
      goal_topic, 1,
      std::bind(&ForkController::OnForkGoal, this, std::placeholders::_1));
  pose_pub_ = nh_->create_publisher<geometry_msgs::msg::PoseStamped>(
      pose_topic, 1);
  completed_pub_ = nh_->create_publisher<std_msgs::msg::Bool>(
      completed_topic, 1);
  weight_pub_ = nh_->create_publisher<std_msgs::msg::Float64>(
      weight_topic, 1);
  contact_left_pub_ = nh_->create_publisher<std_msgs::msg::Bool>(
      contact_left_topic, 1);
  contact_right_pub_ = nh_->create_publisher<std_msgs::msg::Bool>(
      contact_right_topic, 1);
  collision_left_pub_ = nh_->create_publisher<std_msgs::msg::Bool>(
      collision_left_topic, 1);
  collision_right_pub_ = nh_->create_publisher<std_msgs::msg::Bool>(
      collision_right_topic, 1);
  load_state_pub_ = nh_->create_publisher<std_msgs::msg::Bool>(
      load_state_topic, 1);

  update_timer_.SetRate(update_rate_);

  RCLCPP_INFO(rclcpp::get_logger("ForkController"),
              "Initialized fork controller: goal(%s) pose(%s) completed(%s) "
              "goal_tolerance(%.3f) contact_range(%.2f) loaded_weight(%.1f) "
              "auto_attach(%d) attach/detach heights(%.3f/%.3f)",
              goal_topic.c_str(), pose_topic.c_str(), completed_topic.c_str(),
              goal_tolerance_, contact_range_, loaded_weight_,
              auto_attach_ ? 1 : 0, attach_height_, detach_height_);
}

Forklift *ForkController::GetForklift() {
  Forklift *forklift = Forklift::LookupForModel(GetModel());
  if (!forklift && !warned_no_forklift_) {
    warned_no_forklift_ = true;
    RCLCPP_ERROR(rclcpp::get_logger("ForkController"),
                 "model \"%s\" has no Forklift plugin: fork goals are ignored "
                 "and fork/pose stays at 0",
                 GetModel()->GetName().c_str());
  }
  return forklift;
}

LinkAttacher *ForkController::GetAttacher() {
  LinkAttacher *attacher = LinkAttacher::LookupForModel(GetModel());
  if (!attacher && !warned_no_attacher_) {
    warned_no_attacher_ = true;
    RCLCPP_ERROR(rclcpp::get_logger("ForkController"),
                 "model \"%s\" has no LinkAttacher plugin: auto attach, "
                 "contacts and load state are disabled",
                 GetModel()->GetName().c_str());
  }
  return attacher;
}

void ForkController::OnForkGoal(const std_msgs::msg::Float32::SharedPtr msg) {
  Forklift *forklift = GetForklift();
  if (!forklift) return;

  double goal = std::max(0.0, std::min<double>(msg->data,
                                               forklift->lift_height_));
  if ((double)msg->data != goal) {
    RCLCPP_WARN(rclcpp::get_logger("ForkController"),
                "fork goal %.3f m outside [0, %.3f], clamped to %.3f",
                (double)msg->data, forklift->lift_height_, goal);
  }
  // the driver receives periodic republishes of the same goal; only log a
  // change
  if (!has_goal_ || std::fabs(goal - goal_z_) > 1e-6) {
    RCLCPP_INFO(rclcpp::get_logger("ForkController"),
                "fork goal %.3f m (stroke %.3f m)", goal,
                forklift->lift_height_);
  }
  goal_z_ = goal;
  has_goal_ = true;
  forklift->SetTargetElevation(goal_z_);
}

void ForkController::BeforePhysicsStep(const Timekeeper &timekeeper) {
  Forklift *forklift = GetForklift();
  LinkAttacher *attacher = GetAttacher();

  double elevation = forklift ? forklift->current_elevation_ : 0.0;

  if (auto_attach_ && attacher) {
    std::string message;
    if (!attacher->IsAttached() && prev_elevation_ < attach_height_ &&
        elevation >= attach_height_) {
      // forks rose under a load: weld the nearest candidate on (silently a
      // no-op when nothing is in capture range - a plain empty lift)
      if (attacher->Attach("", message)) {
        RCLCPP_INFO(rclcpp::get_logger("ForkController"), "auto attach: %s",
                    message.c_str());
      }
    } else if (attacher->IsAttached() && prev_elevation_ > detach_height_ &&
               elevation <= detach_height_) {
      // forks grounded: the load rests on the floor again
      if (attacher->Detach(message)) {
        RCLCPP_INFO(rclcpp::get_logger("ForkController"), "auto detach: %s",
                    message.c_str());
      }
    }
  }
  prev_elevation_ = elevation;

  if (!update_timer_.CheckUpdate(timekeeper)) {
    return;
  }

  bool attached = attacher && attacher->IsAttached();
  bool contact = attached;
  if (!contact && attacher) {
    std::string message;
    double distance = std::numeric_limits<double>::infinity();
    if (attacher->FindNearestCandidate(message, &distance)) {
      contact = distance <= contact_range_;
    }
  }

  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = timekeeper.GetSimTime();
  pose.header.frame_id = pose_frame_;
  pose.pose.position.z = elevation;
  pose.pose.orientation.w = 1.0;
  pose_pub_->publish(pose);

  // Before the first command, the controller is already at its goal.
  std_msgs::msg::Bool completed;
  completed.data = !has_goal_ || std::fabs(elevation - goal_z_) <= goal_tolerance_;
  completed_pub_->publish(completed);

  std_msgs::msg::Float64 weight;
  weight.data = attached ? loaded_weight_ : 0.0;
  weight_pub_->publish(weight);

  std_msgs::msg::Bool contact_msg;
  contact_msg.data = contact;
  contact_left_pub_->publish(contact_msg);
  contact_right_pub_->publish(contact_msg);

  std_msgs::msg::Bool collision_msg;
  collision_msg.data = false;
  collision_left_pub_->publish(collision_msg);
  collision_right_pub_->publish(collision_msg);

  std_msgs::msg::Bool load_state;
  load_state.data = attached;
  load_state_pub_->publish(load_state);
}
}  // namespace flatland_plugins

PLUGINLIB_EXPORT_CLASS(flatland_plugins::ForkController,
                       flatland_server::ModelPlugin)
