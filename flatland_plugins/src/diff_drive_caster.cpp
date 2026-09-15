// Copyright (c) 2017, Avidbots Corp.
// Copyright (c) 2026, Levent Soysal (Worthle).
// SPDX-License-Identifier: BSD-3-Clause
// Full license notices: LICENSE and SOURCE_NOTICES.

#include <flatland_plugins/diff_drive_caster.h>
#include <flatland_plugins/ros2_compat.h>
#include <flatland_server/debug_visualization.h>
#include <flatland_server/model_plugin.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

namespace flatland_plugins {

// ============================================================================
// Helper Functions
// ============================================================================

static inline float ClampF(float v, float lo, float hi) {
  return std::max(lo, std::min(v, hi));
}

// ============================================================================
// DiffDriveCaster Implementation
// ============================================================================

void DiffDriveCaster::TwistCallback(const geometry_msgs::msg::Twist& msg) {
  twist_msg_ = msg;
}

void DiffDriveCaster::OnInitialize(const YAML::Node& config) {
  tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(nh_);
  YamlReader reader(config);

  // --- Publishing configuration ---
  enable_odom_pub_ = reader.Get<bool>("enable_odom_pub", true);
  enable_odom_tf_pub_ = reader.Get<bool>("enable_odom_tf_pub", true);
  enable_twist_pub_ = reader.Get<bool>("enable_twist_pub", true);
  twist_in_local_frame_ = reader.Get<bool>("twist_in_local_frame", true);

  // --- Body and frame configuration ---
  std::string body_name = reader.Get<std::string>("body");
  std::string odom_frame_id = reader.Get<std::string>("odom_frame_id", "odom");
  std::string ground_truth_frame_id =
      reader.Get<std::string>("ground_truth_frame_id", "map");

  // --- Topic configuration ---
  std::string twist_topic = reader.Get<std::string>("twist_sub", "cmd_vel");
  std::string odom_topic =
      reader.Get<std::string>("odom_pub", "odom");
  std::string ground_truth_topic =
      reader.Get<std::string>("ground_truth_pub", "ground_truth/odom");
  std::string twist_pub_topic = reader.Get<std::string>("twist_pub", "twist");
  std::string pose_topic = reader.Get<std::string>(
      "ground_truth_pose_pub", "ground_truth/pose");

  // --- Noise configuration ---
  std::vector<double> odom_twist_noise =
      reader.GetList<double>("odom_twist_noise", {0, 0, 0}, 3, 3);
  std::vector<double> odom_pose_noise =
      reader.GetList<double>("odom_pose_noise", {0, 0, 0}, 3, 3);

  // --- Publishing rate ---
  double pub_rate =
      reader.Get<double>("pub_rate", std::numeric_limits<double>::infinity());
  update_timer_.SetRate(pub_rate);

  // --- Dynamics constraints ---
  angular_dynamics_.Configure(
      reader.SubnodeOpt("angular_dynamics", YamlReader::MAP).Node());
  linear_dynamics_.Configure(
      reader.SubnodeOpt("linear_dynamics", YamlReader::MAP).Node());

  // --- Caster friction parameters ---
  caster_lat_mu_ = reader.Get<double>("caster_lat_mu", 0.0);  // 0 = full lateral constraint
  caster_long_mu_ = reader.Get<double>("caster_long_mu", 0.0); // 0 = no rolling resistance
  caster_alignment_rate_ = reader.Get<double>("caster_alignment_rate", 8.0);
  if (!std::isfinite(caster_alignment_rate_) || caster_alignment_rate_ <= 0.0) {
    throw YAMLException("caster_alignment_rate must be positive");
  }

  // --- Parse casters configuration ---
  casters_.clear();
  YAML::Node casters_node = reader.SubnodeOpt("casters", YamlReader::LIST).Node();
  if (casters_node && casters_node.IsSequence()) {
    for (const auto& cn : casters_node) {
      YamlReader cr(cn);
      CasterConfig cc;
      cc.body_name = cr.Get<std::string>("body");
      double trail = cr.Get<double>("trail", 0.05);
      // +X points from pivot to wheel contact. The wheel settles opposite
      // the pivot velocity, so this offset trails the direction of travel.
      cc.contact_offset = b2Vec2(static_cast<float>(trail), 0.0f);
      casters_.push_back(cc);
      cr.EnsureAccessedAllKeys();
    }
  }

  // --- Covariance configuration ---
  std::array<double, 36> odom_pose_covar_default = {0};
  odom_pose_covar_default[0] = odom_pose_noise[0];
  odom_pose_covar_default[7] = odom_pose_noise[1];
  odom_pose_covar_default[35] = odom_pose_noise[2];

  std::array<double, 36> odom_twist_covar_default = {0};
  odom_twist_covar_default[0] = odom_twist_noise[0];
  odom_twist_covar_default[7] = odom_twist_noise[1];
  odom_twist_covar_default[35] = odom_twist_noise[2];

  auto odom_twist_covar = reader.GetArray<double, 36>(
      "odom_twist_covariance", odom_twist_covar_default);
  auto odom_pose_covar =
      reader.GetArray<double, 36>("odom_pose_covariance", odom_pose_covar_default);

  reader.EnsureAccessedAllKeys();

  // --- Get base body reference ---
  body_ = GetModel()->GetBody(body_name);
  if (body_ == nullptr) {
    throw YAMLException("Body with name " + Q(body_name) + " does not exist");
  }

  // --- Resolve caster body references ---
  for (auto& cc : casters_) {
    cc.body = GetModel()->GetBody(cc.body_name);
    if (cc.body == nullptr) {
      throw YAMLException("Caster body with name " + Q(cc.body_name) +
                          " does not exist");
    }
  }

  // --- Set up ROS publishers and subscribers ---
  twist_sub_ =
      nh_->create_subscription<geometry_msgs::msg::Twist>(twist_topic, 1, [this](const geometry_msgs::msg::Twist::SharedPtr msg){ TwistCallback(*msg); });

  if (enable_odom_pub_) {
    odom_pub_ = nh_->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 1);
    ground_truth_pub_ =
        nh_->create_publisher<nav_msgs::msg::Odometry>(ground_truth_topic, 1);
    ground_truth_pose_pub_ =
        nh_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(pose_topic, 1);
  }

  if (enable_twist_pub_) {
    twist_pub_ = nh_->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
        twist_pub_topic, 1);
  }

  // --- Initialize messages ---
  ground_truth_msg_.header.frame_id = ground_truth_frame_id;
  ground_truth_msg_.child_frame_id =
      flatland_plugins::resolveTf("", GetModel()->NameSpaceTF(body_->name_));
  ground_truth_msg_.twist.covariance.fill(0);
  ground_truth_msg_.pose.covariance.fill(0);

  odom_msg_ = ground_truth_msg_;
  odom_msg_.header.frame_id = GetModel()->NameSpaceTF(odom_frame_id);

  for (unsigned int i = 0; i < 36; i++) {
    odom_msg_.twist.covariance[i] = odom_twist_covar[i];
    odom_msg_.pose.covariance[i] = odom_pose_covar[i];
  }

  // --- Initialize random number generators ---
  std::random_device rd;
  rng_ = std::default_random_engine(rd());

  for (unsigned int i = 0; i < 3; i++) {
    noise_gen_[i] =
        std::normal_distribution<double>(0.0, sqrt(odom_pose_noise[i]));
  }
  for (unsigned int i = 0; i < 3; i++) {
    noise_gen_[i + 3] =
        std::normal_distribution<double>(0.0, sqrt(odom_twist_noise[i]));
  }

  RCLCPP_INFO(rclcpp::get_logger("DiffDriveCaster"),
                 "Initialized: body=%s, casters=%zu, caster_lat_mu=%.4f, "
                 "caster_long_mu=%.4f",
                 body_->name_.c_str(), casters_.size(),
                 caster_lat_mu_, caster_long_mu_);
}

void DiffDriveCaster::BeforePhysicsStep(const Timekeeper& timekeeper) {
  bool publish = update_timer_.CheckUpdate(timekeeper);

  b2Body* b2body = body_->physics_body_;
  b2Vec2 position = b2body->GetPosition();
  float angle = b2body->GetAngle();

  const double dt = timekeeper.GetStepSize();

  // =========================================================================
  // 1. Apply dynamics limits to commanded velocities
  // =========================================================================
  linear_velocity_cmd_ =
      linear_dynamics_.Limit(linear_velocity_cmd_, twist_msg_.linear.x, dt);
  angular_velocity_cmd_ =
      angular_dynamics_.Limit(angular_velocity_cmd_, twist_msg_.angular.z, dt);

  // =========================================================================
  // 2. SET the commanded velocity on base body (pure simulation, no controller)
  // =========================================================================
  b2Vec2 linear_vel_local(static_cast<float>(linear_velocity_cmd_), 0);
  b2Vec2 linear_vel_world = b2body->GetWorldVector(linear_vel_local);
  float angular_vel = static_cast<float>(angular_velocity_cmd_);

  // Account for center of mass offset: V_cm = V_o + W x r
  b2Vec2 r = b2body->GetWorldCenter() - position;
  b2Vec2 linear_vel_cm = linear_vel_world + angular_vel * b2Vec2(-r.y, r.x);

  b2body->SetLinearVelocity(linear_vel_cm);
  b2body->SetAngularVelocity(angular_vel);

  // =========================================================================
  // 3. Apply caster constraint impulses that MODIFY the velocity
  // =========================================================================
  // Each caster creates an impulse that:
  // - Removes lateral velocity component (perpendicular to caster heading)
  // - Keeps longitudinal velocity component (along caster heading)
  // This simulates the constraint that casters can only roll, not slide sideways
  
  if (!casters_.empty()) {
    float total_mass = b2body->GetMass();
    float total_inertia = b2body->GetInertia();
    
    for (const auto& cc : casters_) {
      b2Body* caster_b2 = cc.body->physics_body_;

      // Get caster's forward direction in world frame
      b2Vec2 caster_forward = caster_b2->GetWorldVector(b2Vec2(1.0f, 0.0f));
      float len = caster_forward.Length();
      if (len > 1e-6f) caster_forward *= (1.0f / len);
      
      // Lateral direction (perpendicular to caster forward)
      b2Vec2 caster_lateral(-caster_forward.y, caster_forward.x);

      // Contact point in world coordinates
      b2Vec2 contact_world = caster_b2->GetWorldPoint(cc.contact_offset);
      
      // Get current velocity of base body at this contact point
      b2Vec2 v_contact = b2body->GetLinearVelocityFromWorldPoint(contact_world);
      
      // Decompose into caster frame
      float v_lat = b2Dot(caster_lateral, v_contact);  // Lateral (sliding) velocity
      float v_long = b2Dot(caster_forward, v_contact); // Longitudinal (rolling) velocity
      
      // Calculate impulse to reduce lateral velocity
      // caster_lat_mu controls how much lateral slip is allowed (0 = full constraint, 1 = no constraint)
      // At caster_lat_mu = 0, we remove 100% of lateral velocity
      // At caster_lat_mu = 1, we remove 0% of lateral velocity
      float lat_removal_factor = 1.0f - static_cast<float>(caster_lat_mu_);
      lat_removal_factor = ClampF(lat_removal_factor, 0.0f, 1.0f);
      
      float target_v_lat = v_lat * (1.0f - lat_removal_factor);  // Reduce lateral velocity
      float delta_v_lat = target_v_lat - v_lat;  // Change needed
      
      // Similarly for longitudinal (rolling resistance)
      float long_removal_factor = static_cast<float>(caster_long_mu_);
      long_removal_factor = ClampF(long_removal_factor, 0.0f, 1.0f);
      
      float target_v_long = v_long * (1.0f - long_removal_factor);
      float delta_v_long = target_v_long - v_long;
      
      // Convert velocity change to impulse
      // For a rigid body: J = m * delta_v (for translation)
      // But we're applying at a contact point, so it also affects rotation
      // Use effective mass at contact point
      b2Vec2 r_contact = contact_world - b2body->GetWorldCenter();
      
      // Effective mass for lateral direction
      float r_cross_n = r_contact.x * caster_lateral.y - r_contact.y * caster_lateral.x;
      float eff_mass_lat = 1.0f / (1.0f / total_mass + r_cross_n * r_cross_n / total_inertia);
      
      // Effective mass for longitudinal direction  
      float r_cross_t = r_contact.x * caster_forward.y - r_contact.y * caster_forward.x;
      float eff_mass_long = 1.0f / (1.0f / total_mass + r_cross_t * r_cross_t / total_inertia);
      
      // Calculate impulses
      float J_lat = eff_mass_lat * delta_v_lat;
      float J_long = eff_mass_long * delta_v_long;
      
      // Apply impulse to base body
      b2Vec2 impulse = J_lat * caster_lateral + J_long * caster_forward;
      b2body->ApplyLinearImpulse(impulse, contact_world, true);
      
      // Rotate caster to trail BEHIND the motion direction.
      // For a trailing caster, the caster's forward (+X) should point
      // OPPOSITE to the velocity at the pivot.
      float speed = v_contact.Length();
      if (speed > 0.05f) {
        // Desired caster angle = direction opposite to velocity
        float desired_angle = atan2f(-v_contact.y, -v_contact.x);
        
        // Compute shortest angular difference
        float current_angle = caster_b2->GetAngle();
        float angle_diff = desired_angle - current_angle;
        // Normalize to [-pi, pi]
        while (angle_diff > M_PI) angle_diff -= 2.0f * M_PI;
        while (angle_diff < -M_PI) angle_diff += 2.0f * M_PI;
        
        // Smoothly rotate toward desired angle using angular velocity
        // Higher gain = faster alignment
        float alignment_rate = static_cast<float>(caster_alignment_rate_);
        caster_b2->SetAngularVelocity(angle_diff * alignment_rate);
      } else {
        // At low speed, let the caster settle (no forced rotation)
        caster_b2->SetAngularVelocity(0.0f);
      }
    }
  }

  // =========================================================================
  // 4. Initialize odometry reference on first step
  // =========================================================================
  if (!initialized_) {
    initial_position_ = position;
    initial_angle_ = angle;
    initialized_ = true;
  }

  // =========================================================================
  // 7. Publish odometry and TF
  // =========================================================================
  if (publish) {
    b2Vec2 linear_vel_local_actual =
        b2body->GetLinearVelocityFromLocalPoint(b2Vec2(0, 0));
    float angular_vel_actual = b2body->GetAngularVelocity();

    // Ground truth message
    ground_truth_msg_.header.stamp = timekeeper.GetSimTime();
    ground_truth_msg_.pose.pose.position.x = position.x - initial_position_.x;
    ground_truth_msg_.pose.pose.position.y = position.y - initial_position_.y;
    ground_truth_msg_.pose.pose.position.z = 0;
    ground_truth_msg_.pose.pose.orientation =
        flatland_plugins::quaternionMsgFromYaw(angle - initial_angle_);

    ground_truth_msg_.twist.twist.linear.z = 0;
    ground_truth_msg_.twist.twist.angular.x = 0;
    ground_truth_msg_.twist.twist.angular.y = 0;

    if (twist_in_local_frame_) {
      ground_truth_msg_.twist.twist.linear.x =
          cos(-angle) * linear_vel_local_actual.x -
          sin(-angle) * linear_vel_local_actual.y;
      ground_truth_msg_.twist.twist.linear.y =
          sin(-angle) * linear_vel_local_actual.x +
          cos(-angle) * linear_vel_local_actual.y;
      ground_truth_msg_.twist.twist.angular.z = angular_vel_actual;
    } else {
      ground_truth_msg_.twist.twist.linear.x = linear_vel_local_actual.x;
      ground_truth_msg_.twist.twist.linear.y = linear_vel_local_actual.y;
      ground_truth_msg_.twist.twist.angular.z = angular_vel_actual;
    }

    // Pose message (absolute world position)
    pose_msg_.header.stamp = timekeeper.GetSimTime();
    pose_msg_.header.frame_id = ground_truth_msg_.header.frame_id;
    pose_msg_.pose.pose.position.x = position.x;
    pose_msg_.pose.pose.position.y = position.y;
    pose_msg_.pose.pose.position.z = 0;
    pose_msg_.pose.pose.orientation = flatland_plugins::quaternionMsgFromYaw(angle);

    // Noisy odometry message
    odom_msg_.header.stamp = timekeeper.GetSimTime();
    odom_msg_.pose.pose = ground_truth_msg_.pose.pose;
    // Odometry starts at the spawn pose and uses its initial heading.
    const double dx = position.x - initial_position_.x;
    const double dy = position.y - initial_position_.y;
    odom_msg_.pose.pose.position.x = cos(initial_angle_) * dx + sin(initial_angle_) * dy;
    odom_msg_.pose.pose.position.y = -sin(initial_angle_) * dx + cos(initial_angle_) * dy;
    ground_truth_msg_.pose.pose.position.x = position.x;
    ground_truth_msg_.pose.pose.position.y = position.y;
    ground_truth_msg_.pose.pose.orientation = flatland_plugins::quaternionMsgFromYaw(angle);

    odom_msg_.twist.twist = ground_truth_msg_.twist.twist;

    odom_msg_.pose.pose.position.x += noise_gen_[0](rng_);
    odom_msg_.pose.pose.position.y += noise_gen_[1](rng_);
    odom_msg_.pose.pose.orientation = flatland_plugins::quaternionMsgFromYaw(
        (angle - initial_angle_) + noise_gen_[2](rng_));

    odom_msg_.twist.twist.linear.x += noise_gen_[3](rng_);
    odom_msg_.twist.twist.linear.y += noise_gen_[4](rng_);
    odom_msg_.twist.twist.angular.z += noise_gen_[5](rng_);

    if (enable_odom_pub_) {
      ground_truth_pub_->publish(ground_truth_msg_);
      odom_pub_->publish(odom_msg_);
      ground_truth_pose_pub_->publish(pose_msg_);
    }

    if (enable_twist_pub_) {
      geometry_msgs::msg::TwistWithCovarianceStamped twist_pub_msg;
      twist_pub_msg.header.stamp = timekeeper.GetSimTime();
      twist_pub_msg.header.frame_id = odom_msg_.child_frame_id;

      twist_pub_msg.twist.twist.linear.x =
          cos(angle) * linear_vel_local_actual.x +
          sin(angle) * linear_vel_local_actual.y + noise_gen_[3](rng_);
      twist_pub_msg.twist.twist.angular.z =
          angular_vel_actual + noise_gen_[5](rng_);
      twist_pub_msg.twist.covariance = odom_msg_.twist.covariance;

      twist_pub_->publish(twist_pub_msg);
    }

    if (enable_odom_tf_pub_) {
      geometry_msgs::msg::TransformStamped odom_tf;
      odom_tf.header = odom_msg_.header;
      odom_tf.child_frame_id = odom_msg_.child_frame_id;
      odom_tf.transform.translation.x = odom_msg_.pose.pose.position.x;
      odom_tf.transform.translation.y = odom_msg_.pose.pose.position.y;
      odom_tf.transform.translation.z = 0;
      odom_tf.transform.rotation = odom_msg_.pose.pose.orientation;
      tf_broadcaster->sendTransform(odom_tf);
    }
  }
}

}  // namespace flatland_plugins

PLUGINLIB_EXPORT_CLASS(flatland_plugins::DiffDriveCaster,
                       flatland_server::ModelPlugin)
