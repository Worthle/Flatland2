// Copyright (c) 2017, Avidbots Corp.
// Copyright (c) 2026, Levent Soysal (Worthle).
// SPDX-License-Identifier: BSD-3-Clause
// Full license notices: LICENSE.

#include <flatland_plugins/mock_detection.h>
#include <flatland_plugins/ros2_compat.h>
#include <flatland_server/exceptions.h>
#include <flatland_server/yaml_reader.h>
#include <tf2/LinearMath/Quaternion.h>
#include <boost/algorithm/string/join.hpp>
#include <cmath>
#include <limits>
#include <pluginlib/class_list_macros.hpp>
#include <set>

using namespace flatland_server;

namespace flatland_plugins {

void MockDetection::OnInitialize(const YAML::Node &config) {
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(nh_);
  ParseParameters(config);

  update_timer_.SetRate(update_rate_);
  detections_publisher_ =
      nh_->create_publisher<geometry_msgs::msg::PoseArray>(topic_, 1);

  resolved_frame_id_ =
      flatland_plugins::resolveTf("", GetModel()->NameSpaceTF(frame_id_));

  // the body to camera transform never changes, construct it once
  tf2::Quaternion q;
  q.setRPY(0, 0, origin_.theta);
  camera_tf_.header.frame_id = flatland_plugins::resolveTf(
      "", GetModel()->NameSpaceTF(body_->GetName()));
  camera_tf_.child_frame_id = resolved_frame_id_;
  camera_tf_.transform.translation.x = origin_.x;
  camera_tf_.transform.translation.y = origin_.y;
  camera_tf_.transform.translation.z = 0;
  camera_tf_.transform.rotation.x = q.x();
  camera_tf_.transform.rotation.y = q.y();
  camera_tf_.transform.rotation.z = q.z();
  camera_tf_.transform.rotation.w = q.w();
}

void MockDetection::BeforePhysicsStep(const Timekeeper &timekeeper) {
  if (!update_timer_.CheckUpdate(timekeeper)) {
    return;
  }

  if (broadcast_tf_) {
    camera_tf_.header.stamp = timekeeper.GetSimTime();
    tf_broadcaster_->sendTransform(camera_tf_);
  }

  // simulate a whole missed detection frame (anchored frames keep going: a
  // dropped frame does not un-anchor the last-seen detections)
  if (dropout_probability_ > 0.0 && dropout_gen_(rng_) < dropout_probability_) {
    BroadcastAnchoredTfs(timekeeper.GetSimTime());
    return;
  }

  // camera pose in the world frame: body world transform composed with the
  // camera mount pose
  const b2Transform &t = body_->GetPhysicsBody()->GetTransform();
  double cam_x = t.p.x + t.q.c * origin_.x - t.q.s * origin_.y;
  double cam_y = t.p.y + t.q.s * origin_.x + t.q.c * origin_.y;
  double cam_yaw = t.q.GetAngle() + origin_.theta;
  double cam_c = cos(cam_yaw), cam_s = sin(cam_yaw);
  b2Vec2 sensor_pos(cam_x, cam_y);

  geometry_msgs::msg::PoseArray detections;

  // walk every body in the physics world and resolve its flatland model; a
  // multi-body model is only considered once (through its first body)
  std::set<Model *> seen_models;
  for (b2Body *b2body = GetModel()->GetPhysicsWorld()->GetBodyList(); b2body;
       b2body = b2body->GetNext()) {
    Body *body = static_cast<Body *>(b2body->GetUserData());
    if (!body) continue;

    Entity *entity = body->GetEntity();
    if (!entity || entity->Type() != Entity::MODEL) continue;

    Model *model = static_cast<Model *>(entity);
    if (model == GetModel()) continue;
    if (!seen_models.insert(model).second) continue;

    const std::string &model_name = model->GetName();
    bool matched = false;
    for (const std::string &prefix : model_prefixes_) {
      if (model_name.compare(0, prefix.size(), prefix) == 0) {
        matched = true;
        break;
      }
    }
    if (!matched) continue;

    // targets lifted out of the camera's vertical view (e.g. a pallet
    // carried on raised forks) are not detectable
    if (body->elevation_ > max_target_elevation_) continue;

    // range and field of view checks in the camera frame
    b2Vec2 target_pos = b2body->GetPosition();
    double dx = target_pos.x - cam_x;
    double dy = target_pos.y - cam_y;
    double distance = std::hypot(dx, dy);
    if (distance < min_range_ || distance > max_range_) continue;

    double local_x = cam_c * dx + cam_s * dy;
    double local_y = -cam_s * dx + cam_c * dy;
    if (std::fabs(std::atan2(local_y, local_x)) > fov_ / 2.0) continue;

    if (!TargetVisible(sensor_pos, model, target_pos)) continue;

    // target pose (model origin shifted by pose_offset in the target frame)
    double target_yaw = b2body->GetAngle();
    double off_c = cos(target_yaw), off_s = sin(target_yaw);
    double target_x =
        target_pos.x + off_c * pose_offset_.x - off_s * pose_offset_.y;
    double target_y =
        target_pos.y + off_s * pose_offset_.x + off_c * pose_offset_.y;

    // express in the camera frame and apply the detection noise
    double rel_x = target_x - cam_x;
    double rel_y = target_y - cam_y;
    geometry_msgs::msg::Pose pose;
    pose.position.x = cam_c * rel_x + cam_s * rel_y + noise_gen_x_(rng_);
    pose.position.y = -cam_s * rel_x + cam_c * rel_y + noise_gen_y_(rng_);
    pose.position.z = 0;

    double pose_yaw =
        target_yaw + pose_offset_.theta - cam_yaw + noise_gen_yaw_(rng_);
    tf2::Quaternion q;
    q.setRPY(0, 0, pose_yaw);
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

    detections.poses.push_back(pose);

    // update this target's anchored frame: the same noisy detection,
    // re-expressed in the world frame so it stays put when the target is no
    // longer seen (instead of riding the robot in the stale camera frame)
    if (broadcast_target_tf_) {
      geometry_msgs::msg::TransformStamped &anchor = anchored_tfs_[model_name];
      anchor.header.frame_id = world_frame_;
      anchor.child_frame_id = flatland_plugins::resolveTf(
          "", GetModel()->NameSpaceTF(target_frame_prefix_ + model_name));
      anchor.transform.translation.x =
          cam_x + cam_c * pose.position.x - cam_s * pose.position.y;
      anchor.transform.translation.y =
          cam_y + cam_s * pose.position.x + cam_c * pose.position.y;
      anchor.transform.translation.z = 0;
      tf2::Quaternion world_q;
      world_q.setRPY(0, 0, cam_yaw + pose_yaw);
      anchor.transform.rotation.x = world_q.x();
      anchor.transform.rotation.y = world_q.y();
      anchor.transform.rotation.z = world_q.z();
      anchor.transform.rotation.w = world_q.w();
    }
  }

  BroadcastAnchoredTfs(timekeeper.GetSimTime());

  // Publish a frame only when at least one target is detected.
  if (detections.poses.empty()) {
    return;
  }

  detections.header.frame_id = resolved_frame_id_;
  detections.header.stamp = timekeeper.GetSimTime();
  detections_publisher_->publish(detections);
}

void MockDetection::BroadcastAnchoredTfs(const rclcpp::Time &stamp) {
  for (auto &kv : anchored_tfs_) {
    kv.second.header.stamp = stamp;
    tf_broadcaster_->sendTransform(kv.second);
  }
}

bool MockDetection::TargetVisible(const b2Vec2 &sensor_pos, Model *target,
                                  const b2Vec2 &target_pos) {
  // Box2D asserts on zero length rays
  if ((target_pos - sensor_pos).LengthSquared() < 1e-6f) {
    return true;
  }

  DetectionRayCallback callback(this, target);
  GetModel()->GetPhysicsWorld()->RayCast(&callback, sensor_pos, target_pos);
  return callback.closest_entity_ == nullptr ||
         callback.closest_entity_ == target;
}

float DetectionRayCallback::ReportFixture(b2Fixture *fixture,
                                          const b2Vec2 &point,
                                          const b2Vec2 &normal,
                                          float fraction) {
  // sensors are not real obstacles
  if (fixture->IsSensor()) return -1.0f;

  // only fixtures on the configured layers occlude the view
  if (!(fixture->GetFilterData().categoryBits & parent_->layers_bits_)) {
    return -1.0f;
  }

  Body *body = static_cast<Body *>(fixture->GetBody()->GetUserData());
  if (!body) return -1.0f;

  // the robot carrying the camera never occludes its own view
  Entity *entity = body->GetEntity();
  if (entity == parent_->GetModel()) return -1.0f;

  if (fraction < closest_fraction_) {
    closest_fraction_ = fraction;
    closest_entity_ = entity;
  }

  // clip the ray so only nearer fixtures keep being reported
  return fraction;
}

void MockDetection::ParseParameters(const YAML::Node &config) {
  YamlReader reader(config);
  std::string body_name = reader.Get<std::string>("body");
  topic_ = reader.Get<std::string>("topic", "detections");
  frame_id_ = reader.Get<std::string>("frame", GetName());
  broadcast_tf_ = reader.Get<bool>("broadcast_tf", true);
  broadcast_target_tf_ = reader.Get<bool>("broadcast_target_tf", false);
  target_frame_prefix_ =
      reader.Get<std::string>("target_frame_prefix", "detected_");
  world_frame_ = reader.Get<std::string>("world_frame", "map");
  update_rate_ = reader.Get<double>("update_rate", 10.0);
  origin_ = reader.GetPose("origin", Pose(0, 0, 0));
  min_range_ = reader.Get<double>("min_range", 0.3);
  max_range_ = reader.Get<double>("max_range", 5.0);
  fov_ = reader.Get<double>("fov", 1.22);
  // targets elevated above this are out of the vertical view; default inf
  // keeps the legacy behavior
  max_target_elevation_ = reader.Get<double>(
      "max_target_elevation", std::numeric_limits<double>::infinity());
  model_prefixes_ = reader.GetList<std::string>("model_prefixes", 1, -1);
  std::vector<double> noise_std_dev =
      reader.GetList<double>("noise_std_dev", {0.0, 0.0, 0.0}, 3, 3);
  dropout_probability_ = reader.Get<double>("dropout_probability", 0.0);
  pose_offset_ = reader.GetPose("pose_offset", Pose(0, 0, 0));

  std::vector<std::string> layers =
      reader.GetList<std::string>("layers", {"all"}, -1, -1);

  reader.EnsureAccessedAllKeys();

  if (fov_ <= 0) {
    throw YAMLException("Invalid \"fov\", must be > 0");
  }

  if (max_range_ <= min_range_) {
    throw YAMLException(
        "Invalid \"min_range\"/\"max_range\", must have max_range > "
        "min_range");
  }

  if (dropout_probability_ < 0 || dropout_probability_ > 1) {
    throw YAMLException(
        "Invalid \"dropout_probability\", must be within [0, 1]");
  }

  body_ = GetModel()->GetBody(body_name);
  if (!body_) {
    throw YAMLException("Cannot find body with name " + body_name);
  }

  std::vector<std::string> invalid_layers;
  layers_bits_ = GetModel()->GetCfr()->GetCategoryBits(layers, &invalid_layers);
  if (!invalid_layers.empty()) {
    throw YAMLException("Cannot find layer(s): {" +
                        boost::algorithm::join(invalid_layers, ",") + "}");
  }

  // init the random number generators
  std::random_device rd;
  rng_ = std::default_random_engine(rd());
  noise_gen_x_ = std::normal_distribution<double>(0.0, noise_std_dev[0]);
  noise_gen_y_ = std::normal_distribution<double>(0.0, noise_std_dev[1]);
  noise_gen_yaw_ = std::normal_distribution<double>(0.0, noise_std_dev[2]);
  dropout_gen_ = std::uniform_real_distribution<double>(0.0, 1.0);

  RCLCPP_INFO(
      rclcpp::get_logger("MockDetection"),
      "MockDetection %s params: topic(%s) body(%s) origin(%f,%f,%f) "
      "frame_id(%s) broadcast_tf(%d) update_rate(%f) range(%f~%f) fov(%f) "
      "model_prefixes({%s}) noise_std_dev(%f,%f,%f) dropout_probability(%f) "
      "layers(0x%u {%s})",
      GetName().c_str(), topic_.c_str(), body_name.c_str(), origin_.x,
      origin_.y, origin_.theta, frame_id_.c_str(), broadcast_tf_, update_rate_,
      min_range_, max_range_, fov_,
      boost::algorithm::join(model_prefixes_, ",").c_str(), noise_std_dev[0],
      noise_std_dev[1], noise_std_dev[2], dropout_probability_, layers_bits_,
      boost::algorithm::join(layers, ",").c_str());
}
};  // namespace flatland_plugins

PLUGINLIB_EXPORT_CLASS(flatland_plugins::MockDetection,
                       flatland_server::ModelPlugin)
