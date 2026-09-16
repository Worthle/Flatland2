// Copyright (c) 2017, Avidbots Corp.
// Copyright (c) 2026, Levent Soysal (Worthle).
// SPDX-License-Identifier: BSD-3-Clause
// Full license notices: LICENSE.

#include <Box2D/Box2D.h>
#include <flatland_plugins/imu.h>

#include <cmath>
#include <flatland_plugins/ros2_compat.h>
#include <flatland_server/debug_visualization.h>
#include <flatland_server/model_plugin.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace flatland_plugins {

void Imu::OnInitialize(const YAML::Node& config) {
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(nh_);
  YamlReader reader(config);
  enable_imu_pub_ = reader.Get<bool>("enable_imu_pub", true);

  std::string body_name = reader.Get<std::string>("body");
  imu_frame_id_ = reader.Get<std::string>("imu_frame_id", "imu");

  std::string imu_topic = reader.Get<std::string>("imu_pub", "imu/filtered");
  std::string ground_truth_topic =
      reader.Get<std::string>("ground_truth_pub", "imu/ground_truth");

  // noise are in the form of linear x, linear y, angular variances
  std::vector<double> orientation_noise =
      reader.GetList<double>("orientation_noise", {0, 0, 0}, 3, 3);
  std::vector<double> angular_velocity_noise =
      reader.GetList<double>("angular_velocity_noise", {0, 0, 0}, 3, 3);
  std::vector<double> linear_acceleration_noise =
      reader.GetList<double>("linear_acceleration_noise", {0, 0, 0}, 3, 3);

  pub_rate_ =
      reader.Get<double>("pub_rate", std::numeric_limits<double>::infinity());
  update_timer_.SetRate(pub_rate_);

  broadcast_tf_ = reader.Get<bool>("broadcast_tf", true);

  // by default the covariance diagonal is the variance of actual noise
  // generated, non-diagonal elements are zero assuming the noises are
  // independent, we also don't care about linear z, angular x, and angular y
  std::array<double, 9> orientation_covar_default = {0};
  orientation_covar_default[0] = orientation_noise[0];
  orientation_covar_default[4] = orientation_noise[1];
  orientation_covar_default[8] = orientation_noise[2];

  std::array<double, 9> angular_velocity_covar_default = {0};
  angular_velocity_covar_default[0] = angular_velocity_noise[0];
  angular_velocity_covar_default[4] = angular_velocity_noise[1];
  angular_velocity_covar_default[8] = angular_velocity_noise[2];

  std::array<double, 9> linear_acceleration_covar_default = {0};
  linear_acceleration_covar_default[0] = linear_acceleration_noise[0];
  linear_acceleration_covar_default[4] = linear_acceleration_noise[1];
  linear_acceleration_covar_default[8] = linear_acceleration_noise[2];

  auto orientation_covar = reader.GetArray<double, 9>(
      "orientation_covariance", orientation_covar_default);

  auto angular_velocity_covar = reader.GetArray<double, 9>(
      "angular_velocity_covariance", angular_velocity_covar_default);

  auto linear_acceleration_covar = reader.GetArray<double, 9>(
      "linear_acceleration_covariance", linear_acceleration_covar_default);

  reader.EnsureAccessedAllKeys();

  body_ = GetModel()->GetBody(body_name);
  if (body_ == nullptr) {
    throw YAMLException("Body with name " + Q(body_name) + " does not exist");
  }

  imu_pub_ = nh_->create_publisher<sensor_msgs::msg::Imu>(imu_topic, 1);
  ground_truth_pub_ = nh_->create_publisher<sensor_msgs::msg::Imu>(ground_truth_topic, 1);

  // init the values for the messages
  ground_truth_msg_.header.frame_id = GetModel()->NameSpaceTF(imu_frame_id_);
  flatland_plugins::resolveTf("", GetModel()->NameSpaceTF(body_->name_));
  ground_truth_msg_.orientation_covariance.fill(0);
  ground_truth_msg_.angular_velocity_covariance.fill(0);
  ground_truth_msg_.linear_acceleration_covariance.fill(0);
  imu_msg_ = ground_truth_msg_;

  // copy from std::array to boost array
  for (int i = 0; i < 9; i++) {
    imu_msg_.orientation_covariance[i] = orientation_covar[i];
    imu_msg_.angular_velocity_covariance[i] = angular_velocity_covar[i];
    imu_msg_.linear_acceleration_covariance[i] = linear_acceleration_covar[i];
  }

  // init the random number generators
  std::random_device rd;
  rng_ = std::default_random_engine(rd());
  for (int i = 0; i < 3; i++) {
    // variance is standard deviation squared
    noise_gen_[i] =
        std::normal_distribution<double>(0.0, sqrt(orientation_noise[i]));
  }
  for (int i = 0; i < 3; i++) {
    // variance is standard deviation squared
    noise_gen_[i + 3] =
        std::normal_distribution<double>(0.0, sqrt(angular_velocity_noise[i]));
  }
  for (int i = 0; i < 3; i++) {
    // variance is standard deviation squared
    noise_gen_[i + 6] = std::normal_distribution<double>(
        0.0, sqrt(linear_acceleration_noise[i]));
  }

  imu_tf_.header.frame_id = flatland_plugins::resolveTf(
      "", GetModel()->NameSpaceTF(body_->GetName()));
  imu_tf_.child_frame_id =
      flatland_plugins::resolveTf("", GetModel()->NameSpaceTF(imu_frame_id_));
  imu_tf_.transform.translation.x = 0;
  imu_tf_.transform.translation.y = 0;  // origin_.y;
  imu_tf_.transform.translation.z = 0;
  imu_tf_.transform.rotation.x = 0;  // q.x();
  imu_tf_.transform.rotation.y = 0;  // q.y();
  imu_tf_.transform.rotation.z = 0;  // q.z();
  imu_tf_.transform.rotation.w = 1;  // q.w();

  RCLCPP_DEBUG(rclcpp::get_logger("Imu"),
      "Initialized with params body(%p %s) imu_frame_id(%s) "
      "imu_pub(%s) ground_truth_pub(%s) "
      "orientation_noise({%f,%f,%f}) angular_velocity_noise({%f,%f,%f}) "
      "linear_acceleration_velocity({%f,%f,%f}) "
      "pub_rate(%f)\n",
      body_, body_->name_.c_str(), imu_frame_id_.c_str(), imu_topic.c_str(),
      ground_truth_topic.c_str(), orientation_noise[0], orientation_noise[1],
      orientation_noise[2], angular_velocity_noise[0],
      angular_velocity_noise[1], angular_velocity_noise[2],
      linear_acceleration_noise[0], linear_acceleration_noise[1],
      linear_acceleration_noise[2], pub_rate_);
}

void Imu::AfterPhysicsStep(const Timekeeper& timekeeper) {
  bool publish = update_timer_.CheckUpdate(timekeeper);

  b2Body* b2body = body_->physics_body_;

  b2Vec2 position = b2body->GetPosition();
  float angle = b2body->GetAngle();
  b2Vec2 linear_vel_local =
      b2body->GetLinearVelocityFromLocalPoint(b2Vec2(0, 0));
  double angular_vel = b2body->GetAngularVelocity();

  if (publish) {
    // Yaw rate from the body's REALIZED rotation since the last sample rather
    // than Box2D's velocity state: the two differ whenever the solver's
    // position correction or a contact constrains the motion (measured
    // 0.158 vs 0.133 rad/s with the truck rubbing an obstacle in a full-lock
    // creep turn). A gyro measures the motion, so report the motion.
    const double now = timekeeper.GetSimTime().seconds();
    if (have_last_angle_ && now > last_angle_time_) {
      double dtheta = angle - last_angle_;
      while (dtheta > M_PI) dtheta -= 2.0 * M_PI;
      while (dtheta < -M_PI) dtheta += 2.0 * M_PI;
      angular_vel = dtheta / (now - last_angle_time_);
    }
    last_angle_ = angle;
    last_angle_time_ = now;
    have_last_angle_ = true;

    // get the state of the body and publish the data

    ground_truth_msg_.header.stamp = timekeeper.GetSimTime();
    ground_truth_msg_.orientation = flatland_plugins::quaternionMsgFromYaw(angle);
    ground_truth_msg_.angular_velocity.z = angular_vel;

    double global_acceleration_x =
        (linear_vel_local.x - linear_vel_local_prev.x) * pub_rate_;
    double global_acceleration_y =
        (linear_vel_local.y - linear_vel_local_prev.y) * pub_rate_;

    ground_truth_msg_.linear_acceleration.x =
        cos(angle) * global_acceleration_x + sin(angle) * global_acceleration_y;
    ground_truth_msg_.linear_acceleration.y =
        sin(angle) * global_acceleration_x + cos(angle) * global_acceleration_y;

    // add the noise to odom messages
    imu_msg_.header.stamp = timekeeper.GetSimTime();

    imu_msg_.orientation =
        flatland_plugins::quaternionMsgFromYaw(angle + noise_gen_[2](rng_));

    imu_msg_.angular_velocity = ground_truth_msg_.angular_velocity;
    imu_msg_.angular_velocity.z += noise_gen_[5](rng_);

    imu_msg_.linear_acceleration = ground_truth_msg_.linear_acceleration;
    imu_msg_.linear_acceleration.x += noise_gen_[6](rng_);
    imu_msg_.linear_acceleration.y += noise_gen_[7](rng_);

    if (enable_imu_pub_) {
      ground_truth_pub_->publish(ground_truth_msg_);
      imu_pub_->publish(imu_msg_);
    }
    linear_vel_local_prev = linear_vel_local;
  }

  if (broadcast_tf_) {
    imu_tf_.header.stamp = timekeeper.GetSimTime();
    tf_broadcaster_->sendTransform(imu_tf_);
  }
}
}

PLUGINLIB_EXPORT_CLASS(flatland_plugins::Imu, flatland_server::ModelPlugin)
