// Copyright (c) 2026, Levent Soysal (Worthle).
// SPDX-License-Identifier: BSD-3-Clause
// Full license notices: LICENSE and SOURCE_NOTICES.

#include <flatland_plugins/link_attacher.h>
#include <flatland_server/exceptions.h>
#include <flatland_server/yaml_reader.h>
#include <pluginlib/class_list_macros.hpp>
#include <boost/algorithm/string/join.hpp>

#include <cmath>
#include <set>
#include <unordered_map>

using namespace flatland_server;

namespace flatland_plugins {

// per-model instance registry so sibling plugins (ForkController) can drive
// attach/detach in-process; init, step and destruction all run on the single
// world-update thread, so no locking is needed
static std::unordered_map<Model *, LinkAttacher *> attacher_registry;

LinkAttacher *LinkAttacher::LookupForModel(Model *model) {
  auto it = attacher_registry.find(model);
  return it == attacher_registry.end() ? nullptr : it->second;
}

void LinkAttacher::OnInitialize(const YAML::Node &config) {
  YamlReader reader(config);

  std::string body_name = reader.Get<std::string>("body");
  model_prefixes_ = reader.GetList<std::string>("model_prefixes", 1, -1);
  std::vector<double> capture_point =
      reader.GetList<double>("capture_point", {0.0, 0.0}, 2, 2);
  capture_range_ = reader.Get<double>("capture_range", 1.0);
  std::string attach_service =
      reader.Get<std::string>("attach_service", "attach");
  std::string detach_service =
      reader.Get<std::string>("detach_service", "detach");
  std::string state_topic =
      reader.Get<std::string>("state_topic", "attached_model");
  std::string elevation_source =
      reader.Get<std::string>("elevation_source_body", "");
  carry_elevation_offset_ =
      reader.Get<double>("carry_elevation_offset", 0.0);
  // a model detached while lifted settles to the ground at this rate (m/s)
  // instead of snapping down in one frame
  drop_speed_ = reader.Get<double>("drop_speed", 0.5);
  update_rate_ = reader.Get<double>("update_rate", 10.0);

  reader.EnsureAccessedAllKeys();

  attach_body_ = GetModel()->GetBody(body_name);
  if (!attach_body_) {
    throw YAMLException("Cannot find body with name " + body_name);
  }

  elevation_source_body_ = nullptr;
  if (!elevation_source.empty()) {
    elevation_source_body_ = GetModel()->GetBody(elevation_source);
    if (!elevation_source_body_) {
      throw YAMLException("Cannot find elevation_source_body with name " +
                          elevation_source);
    }
  }

  capture_point_.Set(capture_point[0], capture_point[1]);

  // a fresh negative group index: fixtures sharing it never collide with
  // each other while all their category/mask bits (sensor visibility, wall
  // collisions) stay intact
  nocollide_group_ = GetModel()->cfr_->RegisterNoCollide();

  state_pub_ = nh_->create_publisher<std_msgs::msg::String>(
      state_topic, 1);
  attach_srv_ = nh_->create_service<flatland_msgs::srv::Attach>(
      attach_service,
      std::bind(&LinkAttacher::OnAttachRequest, this, std::placeholders::_1,
                std::placeholders::_2));
  detach_srv_ = nh_->create_service<std_srvs::srv::Trigger>(
      detach_service,
      std::bind(&LinkAttacher::OnDetachRequest, this, std::placeholders::_1,
                std::placeholders::_2));

  update_timer_.SetRate(update_rate_);
  attacher_registry[GetModel()] = this;

  RCLCPP_INFO(rclcpp::get_logger("LinkAttacher"),
              "Initialized link attacher on body(%s), prefixes({%s}) "
              "capture_point(%f,%f) capture_range(%f) services(%s, %s) "
              "elevation_source(%s)",
              body_name.c_str(),
              boost::algorithm::join(model_prefixes_, ",").c_str(),
              capture_point_.x, capture_point_.y, capture_range_,
              attach_service.c_str(), detach_service.c_str(),
              elevation_source.c_str());
}

LinkAttacher::~LinkAttacher() {
  if (LookupForModel(GetModel()) == this) {
    attacher_registry.erase(GetModel());
  }
  // Intentionally NO physics cleanup here. On world teardown the models,
  // bodies and the b2World are all destroyed BEFORE the plugins (World's
  // destructor body runs first, plugin_manager_ member destructs after), so
  // touching them from this destructor is a use-after-free. On DeleteModel
  // of this model Box2D destroys the weld joint together with the attach
  // body immediately afterwards, so nothing leaks either way.
}

Model *LinkAttacher::FindModelByName(const std::string &name) {
  std::set<Model *> seen;
  for (b2Body *b2body = GetModel()->GetPhysicsWorld()->GetBodyList(); b2body;
       b2body = b2body->GetNext()) {
    Body *body = static_cast<Body *>(b2body->GetUserData());
    if (!body) continue;

    Entity *entity = body->GetEntity();
    if (!entity || entity->Type() != Entity::MODEL) continue;

    Model *model = static_cast<Model *>(entity);
    if (model == GetModel()) continue;
    if (!seen.insert(model).second) continue;

    if (model->GetName() == name) return model;
  }
  return nullptr;
}

Model *LinkAttacher::FindNearestCandidate(std::string &message,
                                          double *distance_out) {
  // world position of the capture point (e.g. the middle of the forks)
  b2Vec2 capture_world =
      attach_body_->GetPhysicsBody()->GetWorldPoint(capture_point_);

  Model *nearest = nullptr;
  double nearest_distance = 0;
  std::set<Model *> seen;
  for (b2Body *b2body = GetModel()->GetPhysicsWorld()->GetBodyList(); b2body;
       b2body = b2body->GetNext()) {
    Body *body = static_cast<Body *>(b2body->GetUserData());
    if (!body) continue;

    Entity *entity = body->GetEntity();
    if (!entity || entity->Type() != Entity::MODEL) continue;

    Model *model = static_cast<Model *>(entity);
    if (model == GetModel()) continue;
    if (!seen.insert(model).second) continue;

    const std::string &model_name = model->GetName();
    bool matched = false;
    for (const std::string &prefix : model_prefixes_) {
      if (model_name.compare(0, prefix.size(), prefix) == 0) {
        matched = true;
        break;
      }
    }
    if (!matched) continue;

    double distance = (b2body->GetPosition() - capture_world).Length();
    if (distance <= capture_range_ &&
        (!nearest || distance < nearest_distance)) {
      nearest = model;
      nearest_distance = distance;
    }
  }

  if (!nearest) {
    message = "no model matching prefixes {" +
              boost::algorithm::join(model_prefixes_, ",") + "} within " +
              std::to_string(capture_range_) + " m of the capture point";
  } else if (distance_out) {
    *distance_out = nearest_distance;
  }
  return nearest;
}

void LinkAttacher::SetCollisionGroup(Model *model, int group) {
  for (Body *body : model->bodies_) {
    for (b2Fixture *f = body->GetPhysicsBody()->GetFixtureList(); f;
         f = f->GetNext()) {
      b2Filter filter = f->GetFilterData();
      filter.groupIndex = group;
      f->SetFilterData(filter);
    }
  }
}

void LinkAttacher::SetModelElevation(Model *model, double elevation) {
  for (Body *body : model->bodies_) {
    body->elevation_ = elevation;
  }
}

bool LinkAttacher::Attach(const std::string &model_name,
                          std::string &message) {
  if (!attached_model_name_.empty()) {
    message =
        "already attached to \"" + attached_model_name_ + "\", detach first";
    return false;
  }

  Model *target = nullptr;
  if (!model_name.empty()) {
    target = FindModelByName(model_name);
    if (!target) {
      message = "model \"" + model_name + "\" does not exist";
      return false;
    }
  } else {
    target = FindNearestCandidate(message);
    if (!target) return false;
  }

  if (target->bodies_.empty()) {
    message = "model \"" + target->GetName() + "\" has no bodies";
    return false;
  }

  // the named path must honor the capture range too: welding a far-away
  // model freezes the current (large) relative pose and turns it into an
  // undrivable outrigger
  b2Vec2 capture_world =
      attach_body_->GetPhysicsBody()->GetWorldPoint(capture_point_);
  double distance =
      (target->bodies_[0]->GetPhysicsBody()->GetPosition() - capture_world)
          .Length();
  if (distance > capture_range_) {
    message = "model \"" + target->GetName() + "\" is " +
              std::to_string(distance) +
              " m from the capture point (capture_range " +
              std::to_string(capture_range_) + ")";
    return false;
  }

  b2World *world = GetModel()->GetPhysicsWorld();
  if (world->IsLocked()) {
    message = "physics world is locked, retry";
    return false;
  }

  // rigid weld holding the current relative pose, anchored at the target
  b2Body *target_b2 = target->bodies_[0]->GetPhysicsBody();
  b2WeldJointDef def;
  def.Initialize(attach_body_->GetPhysicsBody(), target_b2,
                 target_b2->GetPosition());
  def.frequencyHz = 0;
  def.dampingRatio = 0;
  def.collideConnected = false;
  joint_ = world->CreateJoint(&def);

  // stop the two models fighting each other (fork tips inside the pallet
  // channels would otherwise push it away); sensors still see the target
  SetCollisionGroup(GetModel(), nocollide_group_);
  SetCollisionGroup(target, nocollide_group_);

  attached_model_name_ = target->GetName();
  // re-attaching a model that was still settling from a previous detach
  // hands it back to the fork-follow logic
  if (settling_model_ == attached_model_name_) {
    settling_model_.clear();
  }
  message = "attached \"" + attached_model_name_ + "\"";
  RCLCPP_INFO(rclcpp::get_logger("LinkAttacher"), "%s", message.c_str());
  return true;
}

void LinkAttacher::OnAttachRequest(
    const std::shared_ptr<flatland_msgs::srv::Attach::Request> request,
    std::shared_ptr<flatland_msgs::srv::Attach::Response> response) {
  response->success = Attach(request->model_name, response->message);
}

bool LinkAttacher::Detach(std::string &message) {
  if (attached_model_name_.empty()) {
    message = "nothing attached";
    return false;
  }

  b2World *world = GetModel()->GetPhysicsWorld();
  if (world->IsLocked()) {
    message = "physics world is locked, retry";
    return false;
  }

  // the joint may already be gone if the target model was deleted: only
  // destroy it if it is still connected to the attach body
  bool joint_alive = false;
  for (b2JointEdge *edge = attach_body_->GetPhysicsBody()->GetJointList();
       edge; edge = edge->next) {
    if (edge->joint == joint_) {
      joint_alive = true;
      break;
    }
  }
  if (joint_alive) {
    world->DestroyJoint(joint_);
  }
  joint_ = nullptr;

  // restore collisions (all flatland model fixtures use group 0); a model
  // released while lifted settles to the ground at drop_speed instead of
  // snapping down in one frame
  SetCollisionGroup(GetModel(), 0);
  Model *target = FindModelByName(attached_model_name_);
  if (target) {
    SetCollisionGroup(target, 0);
    if (!target->bodies_.empty() && target->bodies_[0]->elevation_ > 0.0) {
      if (!settling_model_.empty() &&
          settling_model_ != attached_model_name_) {
        // only one model settles at a time; ground the previous one
        Model *old_settling = FindModelByName(settling_model_);
        if (old_settling) SetModelElevation(old_settling, 0.0);
      }
      settling_model_ = attached_model_name_;
    }
  }

  message = "detached \"" + attached_model_name_ + "\"";
  attached_model_name_.clear();
  RCLCPP_INFO(rclcpp::get_logger("LinkAttacher"), "%s", message.c_str());
  return true;
}

void LinkAttacher::OnDetachRequest(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void)request;
  response->success = Detach(response->message);
}

void LinkAttacher::BeforePhysicsStep(const Timekeeper &timekeeper) {
  if (!attached_model_name_.empty()) {
    // if the target model gets deleted externally, Box2D destroys the weld
    // joint together with the target's bodies; detect that by checking the
    // attach body's joint list instead of trusting the target name, which a
    // same-name respawn could shadow
    bool joint_alive = false;
    for (b2JointEdge *edge = attach_body_->GetPhysicsBody()->GetJointList();
         edge; edge = edge->next) {
      if (edge->joint == joint_) {
        joint_alive = true;
        break;
      }
    }
    if (!joint_alive) {
      RCLCPP_WARN(rclcpp::get_logger("LinkAttacher"),
                  "attached model \"%s\" disappeared, detaching",
                  attached_model_name_.c_str());
      joint_ = nullptr;
      attached_model_name_.clear();
      SetCollisionGroup(GetModel(), 0);
    } else if (elevation_source_body_) {
      // ride the forks up and down
      Model *target = FindModelByName(attached_model_name_);
      if (target) {
        SetModelElevation(target, elevation_source_body_->elevation_ +
                                      carry_elevation_offset_);
      }
    }
  }

  // a freshly detached lifted model settles to the ground at drop_speed
  if (!settling_model_.empty()) {
    Model *settling = FindModelByName(settling_model_);
    if (!settling || settling->bodies_.empty()) {
      settling_model_.clear();
    } else {
      double elevation = settling->bodies_[0]->elevation_ -
                         drop_speed_ * timekeeper.GetStepSize();
      if (elevation <= 0.0) {
        elevation = 0.0;
        settling_model_.clear();
      }
      SetModelElevation(settling, elevation);
    }
  }

  if (update_timer_.CheckUpdate(timekeeper)) {
    std_msgs::msg::String msg;
    msg.data = attached_model_name_;
    state_pub_->publish(msg);
  }
}
}  // namespace flatland_plugins

PLUGINLIB_EXPORT_CLASS(flatland_plugins::LinkAttacher,
                       flatland_server::ModelPlugin)
