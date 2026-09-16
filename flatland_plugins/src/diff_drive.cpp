// Copyright (c) 2017, Avidbots Corp.
// Copyright (c) 2026, Levent Soysal (Worthle).
// SPDX-License-Identifier: BSD-3-Clause
// Full license notices: LICENSE.

#include <Box2D/Box2D.h>
#include <flatland_plugins/diff_drive.h>
#include <flatland_plugins/ros2_compat.h>
#include <flatland_server/debug_visualization.h>
#include <flatland_server/model_plugin.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace flatland_plugins {

void DiffDrive::TwistCallback(const geometry_msgs::msg::Twist& msg) {
  twist_msg_ = msg;
}

void DiffDrive::OnInitialize(const YAML::Node& config) {
  tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(nh_);
  YamlReader reader(config);
  enable_odom_pub_ = reader.Get<bool>("enable_odom_pub", true);
  enable_odom_tf_pub_ = reader.Get<bool>("enable_odom_tf_pub", true);
  enable_twist_pub_ = reader.Get<bool>("enable_twist_pub", true);
  twist_in_local_frame_ = reader.Get<bool>("twist_in_local_frame", true);

  std::string body_name = reader.Get<std::string>("body");
  std::string odom_frame_id = reader.Get<std::string>("odom_frame_id", "odom");
  std::string ground_truth_frame_id =
      reader.Get<std::string>("ground_truth_frame_id", "map");

  std::string twist_topic = reader.Get<std::string>("twist_sub", "cmd_vel");
  std::string odom_topic =
      reader.Get<std::string>("odom_pub", "odom");
  std::string ground_truth_topic =
      reader.Get<std::string>("ground_truth_pub", "ground_truth/odom");
  std::string twist_pub_topic = reader.Get<std::string>("twist_pub", "twist");
  std::string pose_topic = reader.Get<std::string>("ground_truth_pose_pub", "ground_truth/pose");
  // noise are in the form of linear x, linear y, angular variances
  std::vector<double> odom_twist_noise =
      reader.GetList<double>("odom_twist_noise", {0, 0, 0}, 3, 3);
  std::vector<double> odom_pose_noise =
      reader.GetList<double>("odom_pose_noise", {0, 0, 0}, 3, 3);

  double pub_rate =
      reader.Get<double>("pub_rate", std::numeric_limits<double>::infinity());
  update_timer_.SetRate(pub_rate);

  // Angular dynamics constraints
  angular_dynamics_.Configure(reader.SubnodeOpt("angular_dynamics", YamlReader::MAP).Node());

  // Linear dynamics constraints
  linear_dynamics_.Configure(reader.SubnodeOpt("linear_dynamics", YamlReader::MAP).Node());

  // Identified OE model configuration
  use_id_model_ = reader.Get<bool>("use_id_model", false);
  // Always access the OE model keys so EnsureAccessedAllKeys doesn't complain
  YamlReader lin_oe_reader = reader.SubnodeOpt("linear_oe_model", YamlReader::MAP);
  YamlReader ang_oe_reader = reader.SubnodeOpt("angular_oe_model", YamlReader::MAP);

  if (use_id_model_) {
    std::vector<double> lin_B = lin_oe_reader.GetList<double>("B", 1, -1);
    std::vector<double> lin_F = lin_oe_reader.GetList<double>("F", 1, -1);
    int lin_nk = lin_oe_reader.Get<int>("nk");
    double lin_Ts = lin_oe_reader.Get<double>("Ts");
    linear_oe_model_.Configure(lin_B, lin_F, lin_nk, lin_Ts);

    std::vector<double> ang_B = ang_oe_reader.GetList<double>("B", 1, -1);
    std::vector<double> ang_F = ang_oe_reader.GetList<double>("F", 1, -1);
    int ang_nk = ang_oe_reader.Get<int>("nk");
    double ang_Ts = ang_oe_reader.Get<double>("Ts");
    angular_oe_model_.Configure(ang_B, ang_F, ang_nk, ang_Ts);
    if (std::fabs(lin_Ts - ang_Ts) > 1e-9) {
      throw YAMLException("Linear and angular OE sample times must match");
    }

    oe_time_accumulator_ = 0.0;
    oe_linear_output_ = 0.0;
    oe_angular_output_ = 0.0;

    RCLCPP_INFO(rclcpp::get_logger("flatland"), "DiffDrive: Identified OE models enabled");
  }

  // by default the covariance diagonal is the variance of actual noise
  // generated, non-diagonal elements are zero assuming the noises are
  // independent, we also don't care about linear z, angular x, and angular y
  std::array<double, 36> odom_pose_covar_default = {0};
  odom_pose_covar_default[0] = odom_pose_noise[0];
  odom_pose_covar_default[7] = odom_pose_noise[1];
  odom_pose_covar_default[35] = odom_pose_noise[2];

  std::array<double, 36> odom_twist_covar_default = {0};
  odom_twist_covar_default[0] = odom_twist_noise[0];
  odom_twist_covar_default[7] = odom_twist_noise[1];
  odom_twist_covar_default[35] = odom_twist_noise[2];

  auto odom_twist_covar = reader.GetArray<double, 36>("odom_twist_covariance",
                                                      odom_twist_covar_default);
  auto odom_pose_covar = reader.GetArray<double, 36>("odom_pose_covariance",
                                                     odom_pose_covar_default);

  reader.EnsureAccessedAllKeys();

  body_ = GetModel()->GetBody(body_name);
  if (body_ == nullptr) {
    throw YAMLException("Body with name " + Q(body_name) + " does not exist");
  }

  // publish and subscribe to topics
  twist_sub_ = nh_->create_subscription<geometry_msgs::msg::Twist>(twist_topic, 1, [this](const geometry_msgs::msg::Twist::SharedPtr msg){ TwistCallback(*msg); });
  if (enable_odom_pub_) {
    odom_pub_ = nh_->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 1);
    ground_truth_pub_ =
        nh_->create_publisher<nav_msgs::msg::Odometry>(ground_truth_topic, 1);
    ground_truth_pose_pub_ = nh_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(pose_topic, 1);

  }

  if (enable_twist_pub_) {
    twist_pub_ = nh_->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
        twist_pub_topic, 1);
  }

  // init the values for the messages
  ground_truth_msg_.header.frame_id = ground_truth_frame_id;
  ground_truth_msg_.child_frame_id =
      flatland_plugins::resolveTf("", GetModel()->NameSpaceTF(body_->name_));
  ground_truth_msg_.twist.covariance.fill(0);
  ground_truth_msg_.pose.covariance.fill(0);
  // Odometry message initially is similar to ground truth except for the
  // parent frame ID
  odom_msg_ = ground_truth_msg_;
  odom_msg_.header.frame_id = GetModel()->NameSpaceTF(odom_frame_id);

  // copy from std::array to boost array
  for (unsigned int i = 0; i < 36; i++) {
    odom_msg_.twist.covariance[i] = odom_twist_covar[i];
    odom_msg_.pose.covariance[i] = odom_pose_covar[i];
  }

  // init the random number generators
  std::random_device rd;
  rng_ = std::default_random_engine(rd());
  for (unsigned int i = 0; i < 3; i++) {
    // variance is standard deviation squared
    noise_gen_[i] =
        std::normal_distribution<double>(0.0, sqrt(odom_pose_noise[i]));
  }

  for (unsigned int i = 0; i < 3; i++) {
    noise_gen_[i + 3] =
        std::normal_distribution<double>(0.0, sqrt(odom_twist_noise[i]));
  }

  RCLCPP_DEBUG(rclcpp::get_logger("DiffDrive"),
                  "Initialized with params body(%p %s) odom_frame_id(%s) "
                  "twist_sub(%s) odom_pub(%s) ground_truth_pub(%s) "
                  "odom_pose_noise({%f,%f,%f}) odom_twist_noise({%f,%f,%f}) "
                  "pub_rate(%f)\n",
                  body_, body_->name_.c_str(), odom_frame_id.c_str(),
                  twist_topic.c_str(), odom_topic.c_str(),
                  ground_truth_topic.c_str(), odom_pose_noise[0],
                  odom_pose_noise[1], odom_pose_noise[2], odom_twist_noise[0],
                  odom_twist_noise[1], odom_twist_noise[2], pub_rate);
}

void DiffDrive::BeforePhysicsStep(const Timekeeper& timekeeper) {
  bool publish = update_timer_.CheckUpdate(timekeeper);

  b2Body* b2body = body_->physics_body_;

  b2Vec2 position = b2body->GetPosition();
  float angle = b2body->GetAngle();

  // Apply dynamics limits
  double dt = timekeeper.GetStepSize();

  if (use_id_model_ && linear_oe_model_.IsConfigured() && angular_oe_model_.IsConfigured()) {
    // Use identified OE models
    // The OE models run at their own sample time Ts. Accumulate physics dt
    // and step the model when enough time has elapsed.
    double model_Ts = linear_oe_model_.GetSampleTime();
    oe_time_accumulator_ += dt;

    while (oe_time_accumulator_ >= model_Ts) {
      oe_linear_output_ = linear_oe_model_.Step(twist_msg_.linear.x);
      oe_angular_output_ = angular_oe_model_.Step(twist_msg_.angular.z);
      oe_time_accumulator_ -= model_Ts;
    }

    linear_velocity_ = oe_linear_output_;
    angular_velocity_ = oe_angular_output_;
  } else {
    // Original behavior: apply dynamics limits directly
    linear_velocity_ = linear_dynamics_.Limit(linear_velocity_, twist_msg_.linear.x, dt);
    angular_velocity_ = angular_dynamics_.Limit(angular_velocity_, twist_msg_.angular.z, dt);
  }

  // we apply the twist velocities, this must be done every physics step to make
  // sure Box2D solver applies the correct velocity through out. The velocity
  // given in the twist message should be in the local frame
  b2Vec2 linear_vel_local(linear_velocity_, 0);
  b2Vec2 linear_vel = b2body->GetWorldVector(linear_vel_local);
  float angular_vel = angular_velocity_;  // angular is independent of frames

  // we want the velocity vector in the world frame at the center of mass

  // V_cm = V_o + W x r_cm/o
  // velocity at center of mass equals to the velocity at the body origin plus,
  // angular velocity cross product the displacement from the body origin to the
  // center of mass

  // r is the vector from body origin to the CM in world frame
  b2Vec2 r = b2body->GetWorldCenter() - position;
  b2Vec2 linear_vel_cm = linear_vel + angular_vel * b2Vec2(-r.y, r.x);

  b2body->SetLinearVelocity(linear_vel_cm);
  b2body->SetAngularVelocity(angular_vel);

  if(!initialized_)
  {
    initial_position_ = position;
    initial_angle_ = angle;
    initialized_ = true;
  }
  // Update odom+ground truth messages if needed

  if (publish) {
    // get the state of the body and publish the data
    b2Vec2 linear_vel_local =
        b2body->GetLinearVelocityFromLocalPoint(b2Vec2(0, 0));
    float angular_vel = b2body->GetAngularVelocity();

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
      // change frame of velocity
      ground_truth_msg_.twist.twist.linear.x =
          cos(-angle) * linear_vel_local.x - sin(-angle) * linear_vel_local.y;
      ground_truth_msg_.twist.twist.linear.y =
          sin(-angle) * linear_vel_local.x + cos(-angle) * linear_vel_local.y;
      ground_truth_msg_.twist.twist.angular.z = angular_vel;
    } else {
      ground_truth_msg_.twist.twist.linear.x = linear_vel_local.x;
      ground_truth_msg_.twist.twist.linear.y = linear_vel_local.y;
      ground_truth_msg_.twist.twist.angular.z = angular_vel;
    }
    // Create pose msgs
    pose_msg_.header.stamp = timekeeper.GetSimTime();
    pose_msg_.header.frame_id = ground_truth_msg_.header.frame_id;
    pose_msg_.pose.pose.position.x = position.x;
    pose_msg_.pose.pose.position.y = position.y;
    pose_msg_.pose.pose.position.z = 0;
    pose_msg_.pose.pose.orientation = flatland_plugins::quaternionMsgFromYaw(angle);

    // add the noise to odom messages
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
    odom_msg_.pose.pose.position.x += (noise_gen_[0](rng_));
    odom_msg_.pose.pose.position.y += (noise_gen_[1](rng_));
    odom_msg_.pose.pose.orientation =
        flatland_plugins::quaternionMsgFromYaw((angle - initial_angle_) + noise_gen_[2](rng_));
    odom_msg_.twist.twist.linear.x += noise_gen_[3](rng_);
    odom_msg_.twist.twist.linear.y += noise_gen_[4](rng_);
    odom_msg_.twist.twist.angular.z += noise_gen_[5](rng_);

    if (enable_odom_pub_) {
      ground_truth_pub_->publish(ground_truth_msg_);
      odom_pub_->publish(odom_msg_);
      ground_truth_pose_pub_->publish(pose_msg_);
    }

    if (enable_twist_pub_) {
      // Transform global frame velocity into local frame to simulate encoder
      // readings
      geometry_msgs::msg::TwistWithCovarianceStamped twist_pub_msg;
      twist_pub_msg.header.stamp = timekeeper.GetSimTime();
      twist_pub_msg.header.frame_id = odom_msg_.child_frame_id;

      // Forward velocity in twist.linear.x
      twist_pub_msg.twist.twist.linear.x = cos(angle) * linear_vel_local.x +
                                           sin(angle) * linear_vel_local.y +
                                           noise_gen_[3](rng_);

      // Angular velocity in twist.angular.z
      twist_pub_msg.twist.twist.angular.z = angular_vel + noise_gen_[5](rng_);

      twist_pub_msg.twist.covariance = odom_msg_.twist.covariance;

      twist_pub_->publish(twist_pub_msg);
    }

    if (enable_odom_tf_pub_) {
      // publish odom tf
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
}

PLUGINLIB_EXPORT_CLASS(flatland_plugins::DiffDrive,
                       flatland_server::ModelPlugin)
