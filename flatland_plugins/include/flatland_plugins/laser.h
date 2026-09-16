// Copyright (c) 2017, Avidbots Corp.
// Copyright (c) 2026, Levent Soysal (Worthle).
// SPDX-License-Identifier: BSD-3-Clause
// Full license notices: LICENSE.

#include <flatland_plugins/update_timer.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/timekeeper.h>
#include <flatland_server/types.h>
#include <tf2_ros/transform_broadcaster.h>
#include <thirdparty/ThreadPool.h>
#include <Eigen/Dense>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <set>
#include <thread>
#include <visualization_msgs/msg/marker.hpp>

#ifndef FLATLAND_PLUGINS_LASER_H
#define FLATLAND_PLUGINS_LASER_H

using namespace flatland_server;

namespace flatland_plugins {

/**
 * This class implements the model plugin class and provides laser data
 * for the given configurations
 */
class Laser : public ModelPlugin {
 public:
  std::string topic_;         ///< topic name to publish the laser scan
  Body *body_;                ///<  body the laser frame attaches to
  Pose origin_;               ///< laser frame w.r.t the body
  float range_;               ///< laser max range
  float noise_std_dev_;       ///< noise std deviation
  float max_angle_;           /// < laser max angle
  float min_angle_;           ///< laser min angle
  float increment_;           ///< laser angle increment
  float update_rate_;         ///< the rate laser scan will be published
  std::string frame_id_;      ///< laser frame id name
  bool broadcast_tf_;         ///< whether to broadcast laser origin w.r.t body
  bool always_publish_;       ///< Force publishing even if no subscribers
  bool upside_down_;          ///< whether the lidar is mounted upside down
  float max_body_elevation_;  ///< bodies elevated above this (m) are not hit
                              ///< by the 2D scan (e.g. a lifted pallet)
  uint16_t layers_bits_;  ///< for setting the layers where laser will function
  ThreadPool pool_;       ///< ThreadPool for managing concurrent scan threads
  uint64_t publications_ = 0;
  std::set<b2Body *>
      ignored_bodies_;  ///< Set of bodies to ignore during raycast

  /*
   * for setting reflectance layers. if the laser hits those layers,
   * intensity will be high (255)
   */
  uint16_t reflectance_layers_bits_;

  std::default_random_engine rng_;             ///< random generator
  std::normal_distribution<float> noise_gen_;  ///< gaussian noise generator

  Eigen::Matrix3f m_body_to_laser_;       ///< tf from body to laser
  Eigen::Matrix3f m_world_to_body_;       ///< tf  from world to body
  Eigen::Matrix3f m_world_to_laser_;      ///< tf from world to laser
  Eigen::MatrixXf m_laser_points_;        ///< laser points in the laser' frame
  Eigen::MatrixXf m_world_laser_points_;  /// laser point in the world frame
  Eigen::Vector3f v_zero_point_;          ///< point representing (0,0)
  Eigen::Vector3f v_world_laser_origin_;  ///< (0,0) in the laser frame
  sensor_msgs::msg::LaserScan laser_scan_;  ///< for publishing laser scan
  std::vector<float> m_lastMaxFractions_;   ///< the robot move slowly when
                                            /// comparing with to the scan rate

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr
      scan_publisher_;  ///< ros laser topic publisher
  std::shared_ptr<tf2_ros::TransformBroadcaster>
      tf_broadcaster_;  ///< broadcast laser frame
  geometry_msgs::msg::TransformStamped
      laser_tf_;              ///< tf from body to laser frame
  UpdateTimer update_timer_;  ///< for controlling update rate

  /**
   * @brief Constructor to start the threadpool with N+1 threads
   */
  Laser() : pool_(std::thread::hardware_concurrency() + 1) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("flatland"),
                       "Laser plugin loaded with "
                           << (std::thread::hardware_concurrency() + 1)
                           << " threads");
  };

  /**
   * @brief Initialization for the plugin
   * @param[in] config Plugin YAML Node
   */
  void OnInitialize(const YAML::Node &config) override;

  /**
   * @brief Called when just before physics update
   * @param[in] timekeeper Object managing the simulation time
   */
  void BeforePhysicsStep(const Timekeeper &timekeeper) override;

  /**
   * @brief Method that contains all of the laser range calculations
   */
  void ComputeLaserRanges();

  /**
   * @brief helper function to extract the paramters from the YAML Node
   * @param[in] config Plugin YAML Node
   */
  void ParseParameters(const YAML::Node &config);
};

/**
 * This class handles the b2RayCastCallback ReportFixture method
 * allowing each thread to access its own callback object
 */
class LaserCallback : public b2RayCastCallback {
 public:
  bool did_hit_ = false;  ///< Box2D ray trace checking if ray hits anything
  float fraction_ = 0;    ///< Box2D ray trace fraction
  float intensity_ = 0;   ///< Intensity of raytrace collision
  Laser *parent_;         ///< The parent Laser plugin

  /**
   * Default constructor to assign parent object
   */
  LaserCallback(Laser *parent) : parent_(parent){};

  /**
   * @brief Box2D raytrace call back method required for implementing the
   * b2RayCastCallback abstract class
   * @param[in] fixture Fixture the ray hits
   * @param[in] point Point the ray hits the fixture
   * @param[in] normal Vector indicating the normal at the point hit
   * @param[in] fraction Fraction of ray length at hit point
   */
  float ReportFixture(b2Fixture *fixture, const b2Vec2 &point,
                      const b2Vec2 &normal, float fraction) override;
};
};

#endif
