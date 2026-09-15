// Copyright (c) 2017, Avidbots Corp.
// Copyright (c) 2026, Levent Soysal (Worthle).
// SPDX-License-Identifier: BSD-3-Clause
// Full license notices: LICENSE and SOURCE_NOTICES.

#include <flatland_plugins/model_tf_publisher.h>
#include <flatland_plugins/ros2_compat.h>
#include <flatland_server/exceptions.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/yaml_reader.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <Eigen/Dense>
#include <boost/algorithm/string/join.hpp>

using namespace flatland_server;

namespace flatland_plugins {

void ModelTfPublisher::OnInitialize(const YAML::Node &config) {
  tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(nh_);
  YamlReader reader(config);

  // default values
  publish_tf_world_ = reader.Get<bool>("publish_tf_world", false);
  world_frame_id_ = reader.Get<std::string>("world_frame_id", "map");
  update_rate_ = reader.Get<double>("update_rate",
                                    std::numeric_limits<double>::infinity());

  std::string ref_body_name = reader.Get<std::string>("reference", "");
  std::vector<std::string> excluded_body_names =
      reader.GetList<std::string>("exclude", {}, -1, -1);
  reader.EnsureAccessedAllKeys();

  if (ref_body_name.size() != 0) {
    reference_body_ = GetModel()->GetBody(ref_body_name);

    if (reference_body_ == nullptr) {
      throw YAMLException("Body with name \"" + ref_body_name +
                          "\" does not exist");
    }
  } else {
    // defaults to the first body, the reference body has no effect on the
    // final result, but it changes how the TF would look
    reference_body_ = GetModel()->bodies_[0];
  }

  for (unsigned int i = 0; i < excluded_body_names.size(); i++) {
    Body *body = GetModel()->GetBody(excluded_body_names[i]);

    if (body == nullptr) {
      throw YAMLException("Body with name \"" + excluded_body_names[i] +
                          "\" does not exist");
    } else {
      excluded_bodies_.push_back(body);
    }
  }

  update_timer_.SetRate(update_rate_);

  RCLCPP_DEBUG(rclcpp::get_logger("ModelTfPublisher"),
      "Initialized with params: reference(%s, %p) "
      "publish_tf_world(%d) world_frame_id(%s) update_rate(%f), exclude({%s})",
      reference_body_->name_.c_str(), reference_body_, publish_tf_world_,
      world_frame_id_.c_str(), update_rate_,
      boost::algorithm::join(excluded_body_names, ",").c_str());
}

void ModelTfPublisher::BeforePhysicsStep(const Timekeeper &timekeeper) {
  if (!update_timer_.CheckUpdate(timekeeper)) {
    return;
  }

  Eigen::Matrix3f ref_tf_m;  ///< for storing TF from world to the ref. body
  Eigen::Matrix3f rel_tf;    ///< for storing TF from ref. body to other bodies

  // fill the world to ref. body TF with data from Box2D
  const b2Transform &r = reference_body_->physics_body_->GetTransform();
  ref_tf_m << r.q.c, -r.q.s, r.p.x, r.q.s, r.q.c, r.p.y, 0, 0, 1;

  geometry_msgs::msg::TransformStamped tf_stamped;
  tf_stamped.header.stamp = timekeeper.GetSimTime();

  // loop through the bodies to calculate TF, and ignores excluded bodies
  for (unsigned int i = 0; i < GetModel()->bodies_.size(); i++) {
    Body *body = GetModel()->bodies_[i];
    bool is_excluded = false;

    for (unsigned int j = 0; j < excluded_bodies_.size(); j++) {
      if (body == excluded_bodies_[j]) {
        is_excluded = true;
      }
    }

    if (is_excluded || body == reference_body_) {
      continue;
    }

    // Get transformation of body w.r.t to the world
    const b2Transform &b = body->physics_body_->GetTransform();
    Eigen::Matrix3f body_tf_m;
    body_tf_m << b.q.c, -b.q.s, b.p.x, b.q.s, b.q.c, b.p.y, 0, 0, 1;

    // this calculates the transformation from the reference body to the
    // other body. It is needed because Box2D only provides position and
    // angle of bodies w.r.t to the world
    rel_tf = ref_tf_m.inverse() * body_tf_m;

    // obtain the yaw from the transformation matrice
    double cosine = rel_tf(0, 0);
    double sine = rel_tf(1, 0);
    double yaw = atan2(sine, cosine);

    // publish TF
    tf_stamped.header.frame_id =
        flatland_plugins::resolveTf("", GetModel()->NameSpaceTF(reference_body_->name_));
    tf_stamped.child_frame_id =
        flatland_plugins::resolveTf("", GetModel()->NameSpaceTF(body->name_));
    tf_stamped.transform.translation.x = rel_tf(0, 2);
    tf_stamped.transform.translation.y = rel_tf(1, 2);
    tf_stamped.transform.translation.z = body->elevation_ - reference_body_->elevation_;
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    tf_stamped.transform.rotation.x = q.x();
    tf_stamped.transform.rotation.y = q.y();
    tf_stamped.transform.rotation.z = q.z();
    tf_stamped.transform.rotation.w = q.w();

    tf_broadcaster->sendTransform(tf_stamped);
  }

  // publish world TF if necessary
  if (publish_tf_world_) {
    const b2Vec2 &p = reference_body_->physics_body_->GetPosition();
    double yaw = reference_body_->physics_body_->GetAngle();

    tf_stamped.header.frame_id = world_frame_id_;
    tf_stamped.child_frame_id =
        flatland_plugins::resolveTf("", GetModel()->NameSpaceTF(reference_body_->name_));
    tf_stamped.transform.translation.x = p.x;
    tf_stamped.transform.translation.y = p.y;
    tf_stamped.transform.translation.z = reference_body_->elevation_;
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    tf_stamped.transform.rotation.x = q.x();
    tf_stamped.transform.rotation.y = q.y();
    tf_stamped.transform.rotation.z = q.z();
    tf_stamped.transform.rotation.w = q.w();

    tf_broadcaster->sendTransform(tf_stamped);
  }
}
};

PLUGINLIB_EXPORT_CLASS(flatland_plugins::ModelTfPublisher,
                       flatland_server::ModelPlugin)
