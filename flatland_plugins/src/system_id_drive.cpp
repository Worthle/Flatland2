// Copyright (c) 2017, Avidbots Corp.
// Copyright (c) 2026, Levent Soysal (Worthle).
// SPDX-License-Identifier: BSD-3-Clause
// Full license notices: LICENSE and SOURCE_NOTICES.

#include <flatland_plugins/ros2_compat.h>
#include <flatland_plugins/system_id_drive.h>
#include <flatland_plugins/asset_path.h>
#include <flatland_server/exceptions.h>
#include <flatland_server/yaml_reader.h>
#include <pluginlib/class_list_macros.hpp>
#include <boost/algorithm/string/join.hpp>

#include <cmath>
#include <limits>

using namespace flatland_server;

namespace flatland_plugins {

// message field ids for the command mapping
enum CommandField { FIELD_SPEED = 0, FIELD_STEERING_ANGLE = 1 };

void SystemIdDrive::OnInitialize(const YAML::Node &config) {
  YamlReader r(config);

  std::string body_name = r.Get<std::string>("body");
  std::string model_path = ResolveAssetPath(r.Get<std::string>("model_path"));
  std::string command_sub =
      r.Get<std::string>("command_sub", "ackermann_cmd");
  std::string command_mapping =
      r.Get<std::string>("command_mapping", "ackermann");
  linear_output_ =
      r.Get<std::string>("linear_output", "meas_linear_velocity");
  angular_output_ =
      r.Get<std::string>("angular_output", "meas_angular_velocity");
  cmd_timeout_ = r.Get<double>("cmd_timeout", 0.5);

  std::string odom_frame_id = r.Get<std::string>("odom_frame_id", "odom");
  std::string odom_topic = r.Get<std::string>("odom_pub", "odom");
  std::string ground_truth_topic =
      r.Get<std::string>("ground_truth_pub", "ground_truth/odom");
  std::string ground_truth_frame_id =
      r.Get<std::string>("ground_truth_frame_id", "map");
  std::string pose_topic =
      r.Get<std::string>("ground_truth_pose_pub", "ground_truth/pose");
  double pub_rate =
      r.Get<double>("pub_rate", std::numeric_limits<double>::infinity());

  std::vector<double> odom_twist_noise =
      r.GetList<double>("odom_twist_noise", {0, 0, 0}, 3, 3);
  std::vector<double> odom_pose_noise =
      r.GetList<double>("odom_pose_noise", {0, 0, 0}, 3, 3);

  std::array<double, 36> odom_pose_covar_default = {0};
  odom_pose_covar_default[0] = odom_pose_noise[0];
  odom_pose_covar_default[7] = odom_pose_noise[1];
  odom_pose_covar_default[35] = odom_pose_noise[2];
  std::array<double, 36> odom_twist_covar_default = {0};
  odom_twist_covar_default[0] = odom_twist_noise[0];
  odom_twist_covar_default[7] = odom_twist_noise[1];
  odom_twist_covar_default[35] = odom_twist_noise[2];
  auto odom_twist_covar =
      r.GetArray<double, 36>("odom_twist_covariance", odom_twist_covar_default);
  auto odom_pose_covar =
      r.GetArray<double, 36>("odom_pose_covariance", odom_pose_covar_default);

  r.EnsureAccessedAllKeys();

  body_ = GetModel()->GetBody(body_name);
  if (body_ == nullptr) {
    throw YAMLException("Body with name \"" + body_name + "\" does not exist");
  }

  // load the identified model; fail loudly with the loader's message
  try {
    model_.Load(model_path);
  } catch (const std::exception &e) {
    throw YAMLException(std::string("SystemIdDrive: ") + e.what());
  }

  // resolve which sub-model outputs are the body velocities
  std::vector<std::string> output_names = model_.OutputNames();
  for (size_t i = 0; i < output_names.size(); ++i) {
    if (output_names[i] == linear_output_) linear_index_ = i;
    if (output_names[i] == angular_output_) angular_index_ = i;
  }
  if (linear_index_ < 0 || angular_index_ < 0) {
    throw YAMLException(
        "SystemIdDrive: model outputs {" +
        boost::algorithm::join(output_names, ",") +
        "} do not include linear_output \"" + linear_output_ +
        "\" and angular_output \"" + angular_output_ + "\"");
  }

  // resolve every model command input through the configured message mapping
  if (command_mapping != "ackermann") {
    throw YAMLException("SystemIdDrive: unknown command_mapping \"" +
                        command_mapping + "\" (supported: ackermann)");
  }
  for (const std::string &name : model_.CommandNames()) {
    if (name == "cmd_speed") {
      command_fields_.push_back(FIELD_SPEED);
    } else if (name == "cmd_steering_angle") {
      command_fields_.push_back(FIELD_STEERING_ANGLE);
    } else {
      throw YAMLException(
          "SystemIdDrive: model command input \"" + name +
          "\" is not provided by command_mapping \"ackermann\" "
          "(supported inputs: cmd_speed, cmd_steering_angle)");
    }
  }

  command_sub_ =
      nh_->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
          command_sub, 1,
          [this](const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr
                     msg) { CommandCallback(*msg); });
  odom_pub_ = nh_->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 1);
  ground_truth_pub_ =
      nh_->create_publisher<nav_msgs::msg::Odometry>(ground_truth_topic, 1);
  ground_truth_pose_pub_ =
      nh_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
          pose_topic, 1);

  ground_truth_msg_.header.frame_id = ground_truth_frame_id;
  ground_truth_msg_.child_frame_id = flatland_plugins::resolveTf(
      "", GetModel()->NameSpaceTF(body_->GetName()));
  ground_truth_msg_.twist.covariance.fill(0);
  ground_truth_msg_.pose.covariance.fill(0);
  odom_msg_ = ground_truth_msg_;
  odom_msg_.header.frame_id = GetModel()->NameSpaceTF(odom_frame_id);
  for (unsigned int i = 0; i < 36; i++) {
    odom_msg_.twist.covariance[i] = odom_twist_covar[i];
    odom_msg_.pose.covariance[i] = odom_pose_covar[i];
  }

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

  model_timer_.SetRate(model_.SampleRateHz());
  pub_timer_.SetRate(pub_rate);

  RCLCPP_INFO(
      rclcpp::get_logger("SystemIdDrive"),
      "Initialized SystemIdDrive on body(%s) with model(%s): %zu sub-models "
      "{%s} @ %.1f Hz, commands {%s} from %s (mapping %s, cmd_timeout %.2f "
      "s), odom(%s) ground_truth(%s) pose(%s)",
      body_name.c_str(), model_path.c_str(), model_.SubModels().size(),
      boost::algorithm::join(output_names, ",").c_str(),
      model_.SampleRateHz(),
      boost::algorithm::join(model_.CommandNames(), ",").c_str(),
      command_sub.c_str(), command_mapping.c_str(), cmd_timeout_,
      odom_topic.c_str(), ground_truth_topic.c_str(), pose_topic.c_str());
}

void SystemIdDrive::CommandCallback(
    const ackermann_msgs::msg::AckermannDriveStamped &msg) {
  cmd_speed_ = msg.drive.speed;
  cmd_steering_ = msg.drive.steering_angle;
  cmd_received_ = true;
}

void SystemIdDrive::BeforePhysicsStep(const Timekeeper &timekeeper) {
  b2Body *b2body = body_->physics_body_;
  b2Vec2 position = b2body->GetPosition();
  float angle = b2body->GetAngle();

  if (!initialized_) {
    initial_position_ = position;
    initial_angle_ = angle;
    initialized_ = true;
  }

  // track command freshness in SIM time (the stack is sim-time driven)
  if (cmd_received_) {
    last_cmd_sim_s_ = timekeeper.GetSimTime().seconds();
    cmd_received_ = false;
  }

  // NARX tick at the model's native rate: latch the newest command (ZOH) or
  // zeros when the command stream went silent / never started
  if (model_timer_.CheckUpdate(timekeeper)) {
    bool use_zeros = last_cmd_sim_s_ < 0.0;  // no command ever received
    if (!use_zeros && cmd_timeout_ > 0.0 &&
        timekeeper.GetSimTime().seconds() - last_cmd_sim_s_ > cmd_timeout_) {
      use_zeros = true;
    }

    std::vector<double> commands(command_fields_.size(), 0.0);
    if (!use_zeros) {
      for (size_t i = 0; i < command_fields_.size(); ++i) {
        commands[i] = command_fields_[i] == FIELD_SPEED ? cmd_speed_
                                                        : cmd_steering_;
      }
    }
    std::vector<double> outputs = model_.Step(commands);
    v_hat_ = outputs[linear_index_];
    w_hat_ = outputs[angular_index_];
  }

  // the identified model replaces the propulsion physics: write its
  // velocities onto the body every timestep (held between model ticks)
  b2body->SetLinearVelocity(
      b2Vec2(v_hat_ * cos(angle), v_hat_ * sin(angle)));
  b2body->SetAngularVelocity(w_hat_);

  if (pub_timer_.CheckUpdate(timekeeper)) {
    ground_truth_msg_.header.stamp = timekeeper.GetSimTime();
    ground_truth_msg_.pose.pose.position.x = position.x - initial_position_.x;
    ground_truth_msg_.pose.pose.position.y = position.y - initial_position_.y;
    ground_truth_msg_.pose.pose.position.z = 0;
    ground_truth_msg_.pose.pose.orientation =
        flatland_plugins::quaternionMsgFromYaw(angle - initial_angle_);
    // report the identified model's velocities as the twist
    ground_truth_msg_.twist.twist.linear.x = v_hat_;
    ground_truth_msg_.twist.twist.linear.y = 0;
    ground_truth_msg_.twist.twist.linear.z = 0;
    ground_truth_msg_.twist.twist.angular.x = 0;
    ground_truth_msg_.twist.twist.angular.y = 0;
    ground_truth_msg_.twist.twist.angular.z = w_hat_;

    pose_msg_.header.stamp = timekeeper.GetSimTime();
    pose_msg_.header.frame_id = ground_truth_msg_.header.frame_id;
    pose_msg_.pose.pose.position.x = position.x;
    pose_msg_.pose.pose.position.y = position.y;
    pose_msg_.pose.pose.position.z = 0;
    pose_msg_.pose.pose.orientation =
        flatland_plugins::quaternionMsgFromYaw(angle);

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

    ground_truth_pub_->publish(ground_truth_msg_);
    odom_pub_->publish(odom_msg_);
    ground_truth_pose_pub_->publish(pose_msg_);
  }
}

}  // namespace flatland_plugins

PLUGINLIB_EXPORT_CLASS(flatland_plugins::SystemIdDrive,
                       flatland_server::ModelPlugin)
