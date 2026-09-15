// Copyright (c) 2017, Avidbots Corp.
// Copyright (c) 2026, Levent Soysal (Worthle).
// SPDX-License-Identifier: BSD-3-Clause
// Full license notices: LICENSE and SOURCE_NOTICES.

#include <Box2D/Box2D.h>
#include <cmath>
#include <flatland_plugins/omni_drive.h>
#include <flatland_plugins/ros2_compat.h>
#include <flatland_server/debug_visualization.h>
#include <flatland_server/model_plugin.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <flatland_msgs/msg/channel_values_floating.hpp>

namespace flatland_plugins {

void OmniDrive::Turret1Callback(const ackermann_msgs::msg::AckermannDriveStamped& msg) {
  turret1_.cmd_speed = msg.drive.speed;
  turret1_.cmd_steering = msg.drive.steering_angle;
  RCLCPP_DEBUG_THROTTLE(rclcpp::get_logger("flatland"), *nh_->get_clock(), (1.0)*1000, "Turret1 cmd: speed=%.3f, steering=%.3f rad (%.1f deg)", 
                     turret1_.cmd_speed, turret1_.cmd_steering, turret1_.cmd_steering * 180.0 / M_PI);
}

void OmniDrive::Turret2Callback(const ackermann_msgs::msg::AckermannDriveStamped& msg) {
  // For diagonal turret setup (front-right + rear-left), both wheels should receive
  // their commanded values directly without inversion
  turret2_.cmd_speed = msg.drive.speed;
  turret2_.cmd_steering = msg.drive.steering_angle;
  RCLCPP_DEBUG_THROTTLE(rclcpp::get_logger("flatland"), *nh_->get_clock(), (1.0)*1000, "Turret2 cmd: speed=%.3f, steering=%.3f rad (%.1f deg)", 
                     turret2_.cmd_speed, turret2_.cmd_steering, turret2_.cmd_steering * 180.0 / M_PI);
}

void OmniDrive::OnInitialize(const YAML::Node& config) {
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

  // Turret wheel joint names
  std::string turret1_joint_name = reader.Get<std::string>("turret1_joint");
  std::string turret2_joint_name = reader.Get<std::string>("turret2_joint");
  auto caster_joint_names = reader.GetList<std::string>("caster_joints", {}, 0, -1);
  caster_alignment_rate_ = reader.Get<double>("caster_alignment_rate", 8.0);
  if (!std::isfinite(caster_alignment_rate_) || caster_alignment_rate_ <= 0.0) {
    throw YAMLException("caster_alignment_rate must be finite and positive");
  }

  // Topic names for AckermannDriveStamped commands
  std::string turret1_topic = reader.Get<std::string>("turret1_sub", "ackermann_cmd_1");
  std::string turret2_topic = reader.Get<std::string>("turret2_sub", "ackermann_cmd_2");

  // Topic name for turret angles feedback
  std::string turret_angles_topic = reader.Get<std::string>("turret_angles_pub", "turret_angles/measured");
  std::string wrpms_topic = reader.Get<std::string>("wrpms_pub", "drive/wheel_speeds");

  std::string turret1_cmd_topic = reader.Get<std::string>("turret1_cmd_pub", "drive/turret1/measured");
  std::string turret2_cmd_topic = reader.Get<std::string>("turret2_cmd_pub", "drive/turret2/measured");
  
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

  // Max steering angle (same for both turrets, or configure separately)
  turret1_.max_steer_angle = reader.Get<double>("max_steer_angle", 0.0);
  turret2_.max_steer_angle = turret1_.max_steer_angle;

  // Turret 1 dynamics
  auto turret1_linear_node = reader.SubnodeOpt("turret1_linear_dynamics", YamlReader::MAP);
  turret1_.linear_dynamics.Configure(turret1_linear_node.Node());
  auto turret1_steering_node = reader.SubnodeOpt("turret1_steering_dynamics", YamlReader::MAP);
  turret1_.steering_dynamics.Configure(turret1_steering_node.Node());

  // Turret 2 dynamics
  auto turret2_linear_node = reader.SubnodeOpt("turret2_linear_dynamics", YamlReader::MAP);
  turret2_.linear_dynamics.Configure(turret2_linear_node.Node());
  auto turret2_steering_node = reader.SubnodeOpt("turret2_steering_dynamics", YamlReader::MAP);
  turret2_.steering_dynamics.Configure(turret2_steering_node.Node());

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

  // Get turret joints
  Joint* turret1_joint = GetModel()->GetJoint(turret1_joint_name);
  if (turret1_joint == nullptr) {
    throw YAMLException("Joint with name " + Q(turret1_joint_name) + " does not exist");
  }
  turret1_.wheel_joint = turret1_joint;
  ComputeTurretJoint(turret1_joint, turret1_);

  Joint* turret2_joint = GetModel()->GetJoint(turret2_joint_name);
  if (turret2_joint == nullptr) {
    throw YAMLException("Joint with name " + Q(turret2_joint_name) + " does not exist");
  }
  turret2_.wheel_joint = turret2_joint;
  ComputeTurretJoint(turret2_joint, turret2_);

  for (const auto& name : caster_joint_names) {
    Joint* joint = GetModel()->GetJoint(name);
    if (joint == nullptr || joint->physics_joint_->GetType() != e_revoluteJoint ||
        joint == turret1_joint || joint == turret2_joint) {
      throw YAMLException("Caster joint " + Q(name) + " must be a separate revolute joint");
    }
    auto* pivot = static_cast<b2RevoluteJoint*>(joint->physics_joint_);
    if (pivot->GetBodyA() != body_->physics_body_ &&
        pivot->GetBodyB() != body_->physics_body_) {
      throw YAMLException("Caster joint " + Q(name) + " must attach to the drive body");
    }
    pivot->EnableLimit(false);
    pivot->EnableMotor(false);
    caster_joints_.push_back(pivot);
  }

  // Calculate wheelbase (distance between turrets along body X axis)
  wheelbase_ = fabs(turret1_.pose.x - turret2_.pose.x);
  
  RCLCPP_INFO(rclcpp::get_logger("OmniDrive"), "Turret 1 at (%.3f, %.3f), Turret 2 at (%.3f, %.3f), Wheelbase: %.3f",
                 turret1_.pose.x, turret1_.pose.y, turret2_.pose.x, turret2_.pose.y, wheelbase_);

  // Subscribe to AckermannDriveStamped topics
  turret1_sub_ = nh_->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(turret1_topic, 1, [this](const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg){ Turret1Callback(*msg); });
  turret2_sub_ = nh_->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(turret2_topic, 1, [this](const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg){ Turret2Callback(*msg); });

  // Publisher for turret angles feedback to swerve controller
  // turret_angles_pub_ = nh_->create_publisher<std_msgs::msg::Float64MultiArray>(turret_angles_topic, 1);
  turret_angles_pub_ = nh_->create_publisher<flatland_msgs::msg::ChannelValuesFloating>(turret_angles_topic, 1);
  // Publisher for wheel RPMs
  wrpms_pub_ = nh_->create_publisher<flatland_msgs::msg::ChannelValuesFloating>(wrpms_topic, 1);

  turret1_cmd_pub = nh_->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(turret1_cmd_topic, 1);
  turret2_cmd_pub = nh_->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(turret2_cmd_topic, 1);


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

  RCLCPP_DEBUG(rclcpp::get_logger("OmniDrive"),
                  "Initialized with params body(%p %s) odom_frame_id(%s) "
                  "turret1_sub(%s) turret2_sub(%s) odom_pub(%s) ground_truth_pub(%s) "
                  "odom_pose_noise({%f,%f,%f}) odom_twist_noise({%f,%f,%f}) "
                  "pub_rate(%f)\n",
                  body_, body_->name_.c_str(), odom_frame_id.c_str(),
                  turret1_topic.c_str(), turret2_topic.c_str(), odom_topic.c_str(),
                  ground_truth_topic.c_str(), odom_pose_noise[0],
                  odom_pose_noise[1], odom_pose_noise[2], odom_twist_noise[0],
                  odom_twist_noise[1], odom_twist_noise[2], pub_rate);
}

void OmniDrive::ComputeTurretJoint(Joint* joint, TurretWheel& turret) {
  // Joint must be revolute type for steering
  if (joint->physics_joint_->GetType() != e_revoluteJoint) {
    throw YAMLException("Turret joint " + Q(joint->GetName()) + " must be a revolute joint");
  }

  b2Vec2 wheel_anchor;
  b2Vec2 body_anchor;

  // Determine which body is the main body and which is the wheel
  if (joint->physics_joint_->GetBodyA()->GetUserData() == body_) {
    wheel_anchor = joint->physics_joint_->GetAnchorB();
    body_anchor = joint->physics_joint_->GetAnchorA();
    turret.invert_steering = false;
  } else if (joint->physics_joint_->GetBodyB()->GetUserData() == body_) {
    wheel_anchor = joint->physics_joint_->GetAnchorA();
    body_anchor = joint->physics_joint_->GetAnchorB();
    turret.invert_steering = true;
  } else {
    throw YAMLException("Joint " + Q(joint->GetName()) +
                        " does not anchor on body " + Q(body_->GetName()));
  }

  // Convert anchors to local body coordinates
  body_anchor = body_->physics_body_->GetLocalPoint(body_anchor);
  
  // Store turret position
  turret.pose = body_anchor;

  // Enable limits on the revolute joint for visualization
  b2RevoluteJoint* rev_joint = dynamic_cast<b2RevoluteJoint*>(joint->physics_joint_);
  rev_joint->EnableLimit(true);
}

void OmniDrive::UpdateTurretState(TurretWheel& turret, double dt) {
  // Update steering angle with 2nd-order dynamics (similar to TricycleDriveAckermann)
  double delta_command = turret.cmd_steering;
  
  // Compute commanded steering velocity
  double d_delta_command = 0.0;
  double delta_max_one_step = 0.0;
  
  if (turret.steering_dynamics.acceleration_limit_ > 0.0) {
    delta_max_one_step = turret.steering_velocity * turret.steering_velocity / 
                         (2.0 * turret.steering_dynamics.acceleration_limit_);
  } else {
    delta_max_one_step = fabs(delta_command - turret.current_steering);
  }
  
  if (fabs(delta_command - turret.current_steering) >= delta_max_one_step) {
    d_delta_command = (delta_command - turret.current_steering) / dt;
  }
  
  // Apply steering dynamics
  turret.steering_velocity = turret.steering_dynamics.Limit(
      turret.steering_velocity, d_delta_command, dt);
  
  // Update steering angle
  turret.current_steering += turret.steering_velocity * dt;
  if (turret.max_steer_angle > 0.0) {
    turret.current_steering = DynamicsLimits::Saturate(
        turret.current_steering, -turret.max_steer_angle, turret.max_steer_angle);
  }
  
  // Update speed with dynamics
  turret.current_speed = turret.linear_dynamics.Limit(
      turret.current_speed, turret.cmd_speed, dt);
  
  // Update joint visualization
  if (turret.wheel_joint != nullptr) {
    b2RevoluteJoint* rev_joint = dynamic_cast<b2RevoluteJoint*>(
        turret.wheel_joint->physics_joint_);
    double visual_angle = turret.invert_steering ? -turret.current_steering : turret.current_steering;
    rev_joint->SetLimits(visual_angle, visual_angle);
  }
}

void OmniDrive::ComputeBodyVelocity(double& vx, double& vy, double& omega) {
  // Two turret wheel kinematics for dual-Ackermann setup
  // Each turret contributes velocity at its position based on speed and steering angle
  
  // Turret 1 velocity contribution in body frame
  // The wheel direction is rotated by the steering angle from the body X axis
  double v1 = turret1_.current_speed;
  double theta1 = turret1_.current_steering;
  double v1x = v1 * cos(theta1);
  double v1y = v1 * sin(theta1);
  
  // Turret 2 velocity contribution in body frame  
  double v2 = turret2_.current_speed;
  double theta2 = turret2_.current_steering;
  double v2x = v2 * cos(theta2);
  double v2y = v2 * sin(theta2);
  
  // For a dual-turret robot, we need to compute body velocity from the two wheel velocities
  // Each wheel position: turret1 at (x1, y1), turret2 at (x2, y2) in body frame
  double x1 = turret1_.pose.x;
  double y1 = turret1_.pose.y;
  double x2 = turret2_.pose.x;
  double y2 = turret2_.pose.y;
  
  // The body center velocity can be computed from the two wheel velocities
  // Using rigid body kinematics:
  // v_wheel = v_body + omega × r_wheel
  // 
  // For 2D: 
  // v_wx = vx - omega * ry
  // v_wy = vy + omega * rx
  //
  // From two wheels, we can solve for vx, vy, omega
  
  // If wheels are along X axis (y1 ≈ y2 ≈ 0), the simplified equations:
  // omega = (v1y - v2y) / (x1 - x2)  (from y velocity difference)
  // vx = (v1x + v2x) / 2  (average of x velocities at center)
  // vy = (v1y + v2y) / 2 + omega * (x1 + x2) / 2  (adjusted for rotation)
  
  // More general solution using least squares for over-determined system
  // We have 4 equations (2 per wheel) and 3 unknowns (vx, vy, omega)
  
  // Simplified assumption: wheels are symmetric about the body center on X axis
  // Center is at origin, so body center velocities:
  
  double dx = x1 - x2;  // Distance between wheels in x
  double dy = y1 - y2;  // Distance between wheels in y
  
  if (fabs(dx) > 1e-6) {
    // Wheels separated along X - typical front/rear configuration
    // Angular velocity from the difference in lateral velocities
    omega = (v1y - v2y) / dx;
    
    // Body center velocity (assuming center at origin)
    // v_center = v_wheel1 - omega × r_wheel1
    double rx = x1;  // x1 is already relative to body center
    double ry = y1;
    
    vx = v1x + omega * ry;
    vy = v1y - omega * rx;
    
    // Or average the two solutions for better accuracy
    double vx2 = v2x + omega * y2;
    double vy2 = v2y - omega * x2;
    vx = (vx + vx2) / 2.0;
    vy = (vy + vy2) / 2.0;
    
  } else if (fabs(dy) > 1e-6) {
    // Wheels separated along Y - side-by-side configuration
    omega = -(v1x - v2x) / dy;
    
    vx = (v1x + v2x) / 2.0;
    vy = (v1y + v2y) / 2.0;
  } else {
    // Wheels at same position - shouldn't happen, just average
    vx = (v1x + v2x) / 2.0;
    vy = (v1y + v2y) / 2.0;
    omega = 0.0;
  }
}

void OmniDrive::UpdateCasters() {
  b2Body* base = body_->physics_body_;
  for (auto* joint : caster_joints_) {
    const bool base_is_a = joint->GetBodyA() == base;
    b2Body* caster = base_is_a ? joint->GetBodyB() : joint->GetBodyA();
    const b2Vec2 pivot = base_is_a ? joint->GetAnchorA() : joint->GetAnchorB();
    const b2Vec2 velocity = base->GetLinearVelocityFromWorldPoint(pivot);
    double turn_rate = base->GetAngularVelocity();
    if (velocity.LengthSquared() > 0.0001f) {
      // The wheel contact is offset along caster +X, behind the moving pivot.
      const double target = std::atan2(-velocity.y, -velocity.x);
      const double error = std::remainder(target - caster->GetAngle(), 2.0 * M_PI);
      turn_rate += caster_alignment_rate_ * error;
    }
    caster->SetAngularVelocity(turn_rate);
  }
}

void OmniDrive::BeforePhysicsStep(const Timekeeper& timekeeper) {
  bool publish = update_timer_.CheckUpdate(timekeeper);

  b2Body* b2body = body_->physics_body_;

  b2Vec2 position = b2body->GetPosition();
  float angle = b2body->GetAngle();

  double dt = timekeeper.GetStepSize();

  // Update turret states with dynamics
  UpdateTurretState(turret1_, dt);
  UpdateTurretState(turret2_, dt);

  // Publish current steering angles for swerve controller feedback
  // std_msgs::msg::Float64MultiArray angles_msg;
  flatland_msgs::msg::ChannelValuesFloating angles_msg;
  angles_msg.value.push_back(turret1_.current_steering);
  angles_msg.value.push_back(turret2_.current_steering);
  turret_angles_pub_->publish(angles_msg);

  // Publish wheel RPMs
  flatland_msgs::msg::ChannelValuesFloating wrpms_msg;
  wrpms_msg.value.push_back(turret1_.current_speed);
  wrpms_msg.value.push_back(turret2_.current_speed);
  wrpms_pub_->publish(wrpms_msg);

  // Per-turret measured feedback (AckermannDriveStamped) - the swerve
  // steering controller closes its turret alignment gate with these
  ackermann_msgs::msg::AckermannDriveStamped t1_measured;
  t1_measured.header.stamp = timekeeper.GetSimTime();
  t1_measured.drive.steering_angle = turret1_.current_steering;
  t1_measured.drive.speed = turret1_.current_speed;
  turret1_cmd_pub->publish(t1_measured);

  ackermann_msgs::msg::AckermannDriveStamped t2_measured;
  t2_measured.header.stamp = timekeeper.GetSimTime();
  t2_measured.drive.steering_angle = turret2_.current_steering;
  t2_measured.drive.speed = turret2_.current_speed;
  turret2_cmd_pub->publish(t2_measured);

  // Compute body velocity from the two turret wheels
  double vx_local, vy_local, omega;
  ComputeBodyVelocity(vx_local, vy_local, omega);

  RCLCPP_DEBUG_THROTTLE(rclcpp::get_logger("flatland"), *nh_->get_clock(), (0.5)*1000, "OmniDrive: T1[spd=%.3f, steer=%.1fdeg] T2[spd=%.3f, steer=%.1fdeg] -> body[vx=%.3f, vy=%.3f, omega=%.3f]",
                     turret1_.current_speed, turret1_.current_steering * 180.0 / M_PI,
                     turret2_.current_speed, turret2_.current_steering * 180.0 / M_PI,
                     vx_local, vy_local, omega);

  // Transform local velocity to world frame
  b2Vec2 linear_vel_local(vx_local, vy_local);
  b2Vec2 linear_vel = b2body->GetWorldVector(linear_vel_local);

  // Apply velocity at center of mass
  // V_cm = V_o + W x r_cm/o
  b2Vec2 r = b2body->GetWorldCenter() - position;
  b2Vec2 linear_vel_cm = linear_vel + omega * b2Vec2(-r.y, r.x);

  b2body->SetLinearVelocity(linear_vel_cm);
  b2body->SetAngularVelocity(omega);
  UpdateCasters();

  if(!initialized_)
  {
    initial_position_ = position;
    initial_angle_ = angle;
    initialized_ = true;
  }

  // Update odom+ground truth messages if needed
  if (publish) {
    // get the state of the body and publish the data
    b2Vec2 linear_vel_local_measured =
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
          cos(-angle) * linear_vel_local_measured.x - sin(-angle) * linear_vel_local_measured.y;
      ground_truth_msg_.twist.twist.linear.y =
          sin(-angle) * linear_vel_local_measured.x + cos(-angle) * linear_vel_local_measured.y;
      ground_truth_msg_.twist.twist.angular.z = angular_vel;
    } else {
      ground_truth_msg_.twist.twist.linear.x = linear_vel_local_measured.x;
      ground_truth_msg_.twist.twist.linear.y = linear_vel_local_measured.y;
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
      // readings for omnidirectional drive
      geometry_msgs::msg::TwistWithCovarianceStamped twist_pub_msg;
      twist_pub_msg.header.stamp = timekeeper.GetSimTime();
      twist_pub_msg.header.frame_id = odom_msg_.child_frame_id;

      // Local frame velocities
      twist_pub_msg.twist.twist.linear.x = cos(angle) * linear_vel_local_measured.x +
                                           sin(angle) * linear_vel_local_measured.y +
                                           noise_gen_[3](rng_);
      twist_pub_msg.twist.twist.linear.y = -sin(angle) * linear_vel_local_measured.x +
                                           cos(angle) * linear_vel_local_measured.y +
                                           noise_gen_[4](rng_);
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
}  // void OmniDrive::BeforePhysicsStep
}  // namespace flatland_plugins

PLUGINLIB_EXPORT_CLASS(flatland_plugins::OmniDrive,
                       flatland_server::ModelPlugin)
