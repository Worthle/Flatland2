// Copyright (c) 2017, Avidbots Corp.
// Copyright (c) 2026, Levent Soysal (Worthle).
// SPDX-License-Identifier: BSD-3-Clause
// Full license notices: LICENSE and SOURCE_NOTICES.

#ifndef FLATLAND_PLUGINS_MOCK_LIDAR3D_H
#define FLATLAND_PLUGINS_MOCK_LIDAR3D_H

#include <flatland_plugins/update_timer.h>
#include <flatland_server/model.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/types.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <flatland_plugins/pcd_sampler.h>
#include <random>
#include <string>
#include <vector>

using namespace flatland_server;

namespace flatland_plugins {

/**
 * Samples a reference PCD with a spherical nearest-return projection into
 * azimuth/elevation bins. Dynamic bodies contribute approximate returns at
 * their configured elevation and extrusion height. This is a sensor mock,
 * not full 3D ray tracing. An empty pcd_path enables 2D-wall extrusion mode
 * with configurable heights and an optional synthetic ceiling.
 */
class MockLidar3D : public ModelPlugin {
 public:
  std::string topic_;     ///< topic to publish the PointCloud2 on
  Body *body_;            ///< body the lidar is mounted on
  Pose origin_;           ///< lidar pose w.r.t the body (x, y, yaw)
  double origin_z_;       ///< lidar mount height above the floor (m)
  std::string frame_id_;  ///< lidar frame id
  bool broadcast_tf_;     ///< whether to broadcast lidar frame w.r.t body
  double update_rate_;    ///< publish rate (Hz)
  double min_range_;      ///< minimum return range (m)
  double max_range_;      ///< maximum return range (m)
  int num_rays_;          ///< azimuth bins over 360 degrees
  uint16_t layers_bits_;  ///< layers the dynamic-object rays collide with
  double range_noise_std_dev_;  ///< gaussian noise on the measured range

  // pcd sampling mode
  std::string pcd_path_;           ///< reference 3D map; empty = legacy mode
  std::vector<float> map_pts_;     ///< flattened x,y,z of the map points
  std::vector<double> elevations_;      ///< beam elevation channels (rad)
  double elev_tolerance_;               ///< max |elev - channel| to accept
  double default_object_height_;   ///< dyn body height when extrude is 0

  // legacy extrusion mode (pcd_path empty)
  std::vector<double> heights_;  ///< z heights each 2D hit is replicated at
  double ceiling_height_;     ///< z of the synthetic ceiling plane (0 = off)
  double ceiling_max_range_;  ///< radius the ceiling is sampled out to
  double ceiling_ring_step_;  ///< radial spacing of the ceiling sample rings
  int ceiling_azimuths_;      ///< azimuth samples per ceiling ring

  std::default_random_engine rng_;                    ///< random generator
  std::normal_distribution<double> range_noise_gen_;  ///< range noise

  std::vector<double> ray_cos_;  ///< precomputed cos per azimuth (lidar frame)
  std::vector<double> ray_sin_;  ///< precomputed sin per azimuth (lidar frame)

  UpdateTimer update_timer_;  ///< for controlling the update rate
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      cloud_publisher_;  ///< PointCloud2 publisher (sensor data QoS)
  std::shared_ptr<tf2_ros::TransformBroadcaster>
      tf_broadcaster_;                             ///< broadcast lidar frame
  geometry_msgs::msg::TransformStamped lidar_tf_;  ///< body to lidar frame
  std::string resolved_frame_id_;  ///< namespaced frame id used in headers

  /**
   * @brief Initialization for the plugin
   * @param[in] config Plugin YAML Node
   */
  void OnInitialize(const YAML::Node &config) override;

  /**
   * @brief Sample the world and publish the point cloud
   * @param[in] timekeeper Object managing the simulation time
   */
  void BeforePhysicsStep(const Timekeeper &timekeeper) override;

 private:
  int sampling_threads_ = 0;
  std::unique_ptr<PcdSampler> pcd_sampler_;
  PcdReturns dynamic_returns_{0};
  rclcpp::Time scan_stamp_{0, 0, RCL_ROS_TIME};
  // Destroy the async future before its sampler and immutable map storage.
  std::future<PcdReturns> pending_scan_;

  /**
   * @brief helper function to extract the parameters from the YAML Node
   * @param[in] config Plugin YAML Node
   */
  void ParseParameters(const YAML::Node &config);

  /**
   * @brief Load the reference map .pcd (ascii or binary, x y z [...])
   */
  void LoadPcd(const std::string &path);

  /**
   * @brief Publish the given sensor-frame points as a PointCloud2
   */
  void PublishCloud(const std::vector<std::array<float, 3>> &pts,
                    const rclcpp::Time &stamp);

  /**
   * @brief pcd mode: spherical z-buffer over the map + 2.5D dynamic bodies
   */
  void SamplePcdMode(const Timekeeper &timekeeper);
  void FinishPcdScan();

  /**
   * @brief legacy mode: extrude the 2D wall raycast to fixed heights
   */
  void SampleExtrusionMode(const Timekeeper &timekeeper);
};

/**
 * Raycast callback keeping the closest hit. In pcd mode it only accepts
 * dynamic bodies (MODEL entities, static map layers are already in the
 * pcd); in legacy mode it accepts everything on the configured layers.
 */
class Lidar3DRayCallback : public b2RayCastCallback {
 public:
  MockLidar3D *parent_;            ///< the parent MockLidar3D plugin
  bool models_only_;               ///< accept only MODEL entities
  bool did_hit_ = false;           ///< whether anything was hit
  float closest_fraction_ = 1.0f;  ///< ray fraction of the closest hit
  Body *hit_body_ = nullptr;       ///< flatland body of the closest hit

  Lidar3DRayCallback(MockLidar3D *parent, bool models_only)
      : parent_(parent), models_only_(models_only) {}

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
};  // namespace flatland_plugins

#endif  // FLATLAND_PLUGINS_MOCK_LIDAR3D_H
