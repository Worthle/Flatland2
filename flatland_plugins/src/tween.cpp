// Copyright (c) 2017, Avidbots Corp.
// Copyright (c) 2026, Levent Soysal (Worthle).
// SPDX-License-Identifier: BSD-3-Clause
// Full license notices: LICENSE.

#include <Box2D/Box2D.h>
#include <flatland_plugins/tween.h>
#include <flatland_server/debug_visualization.h>
#include <flatland_server/model_plugin.h>
#include <tf2/utils.h>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace flatland_plugins {

std::map<std::string, Tween::ModeType_> Tween::mode_strings_ = {
    {"yoyo", Tween::ModeType_::YOYO},
    {"loop", Tween::ModeType_::LOOP},
    {"once", Tween::ModeType_::ONCE},
    {"trigger", Tween::ModeType_::TRIGGER}};

std::map<std::string, Tween::EasingType_> Tween::easing_strings_ = {
    {"linear", Tween::EasingType_::linear},
    {"quadraticIn", Tween::EasingType_::quadraticIn},
    {"quadraticOut", Tween::EasingType_::quadraticOut},
    {"quadraticInOut", Tween::EasingType_::quadraticInOut},
    {"cubicIn", Tween::EasingType_::cubicIn},
    {"cubicOut", Tween::EasingType_::cubicOut},
    {"cubicInOut", Tween::EasingType_::cubicInOut},
    {"quarticIn", Tween::EasingType_::quarticIn},
    {"quarticOut", Tween::EasingType_::quarticOut},
    {"quarticInOut", Tween::EasingType_::quarticInOut},
    {"quinticIn", Tween::EasingType_::quinticIn},
    {"quinticOut", Tween::EasingType_::quinticOut},
    {"quinticInOut", Tween::EasingType_::quinticInOut},
    // { "sinuisodal", Tween::EasingType_::sinuisodal },
    {"exponentialIn", Tween::EasingType_::exponentialIn},
    {"exponentialOut", Tween::EasingType_::exponentialOut},
    {"exponentialInOut", Tween::EasingType_::exponentialInOut},
    {"circularIn", Tween::EasingType_::circularIn},
    {"circularOut", Tween::EasingType_::circularOut},
    {"circularInOut", Tween::EasingType_::circularInOut},
    {"backIn", Tween::EasingType_::backIn},
    {"backOut", Tween::EasingType_::backOut},
    {"backInOut", Tween::EasingType_::backInOut},
    {"elasticIn", Tween::EasingType_::elasticIn},
    {"elasticOut", Tween::EasingType_::elasticOut},
    {"elasticInOut", Tween::EasingType_::elasticInOut},
    {"bounceIn", Tween::EasingType_::bounceIn},
    {"bounceOut", Tween::EasingType_::bounceOut},
    {"bounceInOut", Tween::EasingType_::bounceInOut}};

void Tween::OnInitialize(const YAML::Node& config) {
  YamlReader reader(config);
  std::string body_name = reader.Get<std::string>("body");

  // reciprocal, loop, or oneshot
  std::string mode = reader.Get<std::string>("mode", "yoyo");
  duration_ = reader.Get<float>("duration", 1.0);

  delta_ = reader.GetPose("delta", Pose(0, 0, 0));

  // Boolean play pause topic
  std::string trigger_topic = reader.Get<std::string>("trigger_topic", "");
  if (trigger_topic != "") {
    trigger_sub_ = nh_->create_subscription<std_msgs::msg::Bool>(
        trigger_topic, 1, [this](const std_msgs::msg::Bool::SharedPtr msg) {
          TriggerCallback(*msg);
        });
  }

  body_ = GetModel()->GetBody(body_name);
  if (body_ == nullptr) {
    throw YAMLException("Body with name " + Q(body_name) + " does not exist");
  }
  start_ = Pose(body_->physics_body_->GetPosition().x,
                body_->physics_body_->GetPosition().y,
                body_->physics_body_->GetAngle());

  // Validate the mode selection
  if (!Tween::mode_strings_.count(mode)) {
    throw YAMLException("Mode " + mode + " does not exist");
  }
  mode_ = Tween::mode_strings_.at(mode);

  tween_ = tweeny::from(0.0, 0.0, 0.0)
               .to(delta_.x, delta_.y, delta_.theta)
               .during((uint32)(duration_ * 1000.0));

  Tween::EasingType_ easing_type;
  std::string easing = reader.Get<std::string>("easing", "linear");
  if (!Tween::easing_strings_.count(easing)) {
    throw YAMLException("Mode " + mode + " does not exist");
  }
  easing_type = Tween::easing_strings_.at(easing);

  // This is clumsy but because tweeny used structs for each tweening rather
  // than subclasses
  // I believe that this is the best way to do this
  switch (easing_type) {
    case Tween::EasingType_::linear:
      tween_ = tween_.via(tweeny::easing::linear);
      break;
    case Tween::EasingType_::quadraticIn:
      tween_ = tween_.via(tweeny::easing::quadraticIn);
      break;
    case Tween::EasingType_::quadraticOut:
      tween_ = tween_.via(tweeny::easing::quadraticOut);
      break;
    case Tween::EasingType_::quadraticInOut:
      tween_ = tween_.via(tweeny::easing::quadraticInOut);
      break;
    case Tween::EasingType_::cubicIn:
      tween_ = tween_.via(tweeny::easing::cubicIn);
      break;
    case Tween::EasingType_::cubicOut:
      tween_ = tween_.via(tweeny::easing::cubicOut);
      break;
    case Tween::EasingType_::cubicInOut:
      tween_ = tween_.via(tweeny::easing::cubicInOut);
      break;
    case Tween::EasingType_::quarticIn:
      tween_ = tween_.via(tweeny::easing::quarticIn);
      break;
    case Tween::EasingType_::quarticOut:
      tween_ = tween_.via(tweeny::easing::quarticOut);
      break;
    case Tween::EasingType_::quarticInOut:
      tween_ = tween_.via(tweeny::easing::quarticInOut);
      break;
    case Tween::EasingType_::quinticIn:
      tween_ = tween_.via(tweeny::easing::quinticIn);
      break;
    case Tween::EasingType_::quinticOut:
      tween_ = tween_.via(tweeny::easing::quinticOut);
      break;
    case Tween::EasingType_::quinticInOut:
      tween_ = tween_.via(tweeny::easing::quinticInOut);
      break;
    // case Tween::EasingType_::sinuisodal:
    //   tween_ = tween_.via(tweeny::easing::sinuisodal);
    //   break;
    case Tween::EasingType_::exponentialIn:
      tween_ = tween_.via(tweeny::easing::exponentialIn);
      break;
    case Tween::EasingType_::exponentialOut:
      tween_ = tween_.via(tweeny::easing::exponentialOut);
      break;
    case Tween::EasingType_::exponentialInOut:
      tween_ = tween_.via(tweeny::easing::exponentialInOut);
      break;
    case Tween::EasingType_::circularIn:
      tween_ = tween_.via(tweeny::easing::circularIn);
      break;
    case Tween::EasingType_::circularOut:
      tween_ = tween_.via(tweeny::easing::circularOut);
      break;
    case Tween::EasingType_::circularInOut:
      tween_ = tween_.via(tweeny::easing::circularInOut);
      break;
    case Tween::EasingType_::backIn:
      tween_ = tween_.via(tweeny::easing::backIn);
      break;
    case Tween::EasingType_::backOut:
      tween_ = tween_.via(tweeny::easing::backOut);
      break;
    case Tween::EasingType_::backInOut:
      tween_ = tween_.via(tweeny::easing::backInOut);
      break;
    case Tween::EasingType_::elasticIn:
      tween_ = tween_.via(tweeny::easing::elasticIn);
      break;
    case Tween::EasingType_::elasticOut:
      tween_ = tween_.via(tweeny::easing::elasticOut);
      break;
    case Tween::EasingType_::elasticInOut:
      tween_ = tween_.via(tweeny::easing::elasticInOut);
      break;
    case Tween::EasingType_::bounceIn:
      tween_ = tween_.via(tweeny::easing::bounceIn);
      break;
    case Tween::EasingType_::bounceOut:
      tween_ = tween_.via(tweeny::easing::bounceOut);
      break;
    case Tween::EasingType_::bounceInOut:
      tween_ = tween_.via(tweeny::easing::bounceInOut);
      break;
    default:
      throw new Exception("Unknown easing type!");
  }

  // Make sure there are no unused keys
  reader.EnsureAccessedAllKeys();

  RCLCPP_DEBUG(rclcpp::get_logger("Tween"),
               "Initialized with params body(%p %s) "
               "start ({%f,%f,%f}) "
               "end ({%f,%f,%f}) "
               "duration %f "
               "mode: %s [%d] "
               "easing: %s\n",
               body_, body_->name_.c_str(), start_.x, start_.y, start_.theta,
               delta_.x, delta_.y, delta_.theta, duration_, mode.c_str(),
               (int)mode_, easing.c_str());
}

void Tween::TriggerCallback(const std_msgs::msg::Bool& msg) {
  triggered_ = msg.data;
}

void Tween::BeforePhysicsStep(const Timekeeper& timekeeper) {
  std::array<double, 3> v =
      tween_.step((uint32)(timekeeper.GetStepSize() * 1000.0));
  RCLCPP_DEBUG_THROTTLE(rclcpp::get_logger("Tween"), *nh_->get_clock(),
                        (1.0) * 1000, "value %f,%f,%f step %f progress %f",
                        v[0], v[1], v[2], timekeeper.GetStepSize(),
                        tween_.progress());
  body_->physics_body_->SetTransform(b2Vec2(start_.x + v[0], start_.y + v[1]),
                                     start_.theta + v[2]);
  // Tell Box2D to update the AABB and check for collisions for this object
  body_->physics_body_->SetAwake(true);

  // Yoyo back and forth
  if (mode_ == Tween::ModeType_::YOYO) {
    if (tween_.progress() >= 1.0f) {
      tween_.backward();
    } else if (tween_.progress() <= 0.001f) {
      tween_.forward();
    }
  }

  // Teleport back in loop mode
  if (mode_ == Tween::ModeType_::LOOP) {
    if (tween_.progress() >= 1.0f) {
      tween_.seek(0);
    }
  }

  // Handle external trigger
  if (mode_ == Tween::ModeType_::TRIGGER) {
    if (triggered_) {
      tween_.forward();
    } else {
      tween_.backward();
    }
  }
}
}

PLUGINLIB_EXPORT_CLASS(flatland_plugins::Tween, flatland_server::ModelPlugin)
