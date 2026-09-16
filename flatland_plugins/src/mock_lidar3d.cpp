// Copyright (c) 2017, Avidbots Corp.
// Copyright (c) 2026, Levent Soysal (Worthle).
// SPDX-License-Identifier: BSD-3-Clause
// Full license notices: LICENSE.

#include <flatland_plugins/asset_path.h>
#include <flatland_plugins/mock_lidar3d.h>
#include <flatland_plugins/ros2_compat.h>
#include <flatland_server/exceptions.h>
#include <flatland_server/yaml_reader.h>
#include <tf2/LinearMath/Quaternion.h>
#include <array>
#include <boost/algorithm/string/join.hpp>
#include <cmath>
#include <fstream>
#include <limits>
#include <pluginlib/class_list_macros.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sstream>

using namespace flatland_server;

namespace flatland_plugins {

void MockLidar3D::OnInitialize(const YAML::Node &config) {
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(nh_);
  ParseParameters(config);

  update_timer_.SetRate(update_rate_);
  cloud_publisher_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>(
      topic_, rclcpp::SensorDataQoS().keep_last(1));

  resolved_frame_id_ =
      flatland_plugins::resolveTf("", GetModel()->NameSpaceTF(frame_id_));

  if (!pcd_path_.empty()) {
    LoadPcd(pcd_path_);
    const size_t threads =
        sampling_threads_ > 0
            ? sampling_threads_
            : std::min(8u, std::max(1u, std::thread::hardware_concurrency()));
    pcd_sampler_ = std::make_unique<PcdSampler>(
        map_pts_, elevations_, elev_tolerance_, num_rays_, min_range_,
        max_range_, threads);
  }

  // precompute the azimuth directions in the lidar frame
  ray_cos_.resize(num_rays_);
  ray_sin_.resize(num_rays_);
  for (int i = 0; i < num_rays_; i++) {
    double angle = -M_PI + i * (2.0 * M_PI / num_rays_);
    ray_cos_[i] = cos(angle);
    ray_sin_[i] = sin(angle);
  }

  // the body to lidar transform never changes, construct it once
  tf2::Quaternion q;
  q.setRPY(0, 0, origin_.theta);
  lidar_tf_.header.frame_id = flatland_plugins::resolveTf(
      "", GetModel()->NameSpaceTF(body_->GetName()));
  lidar_tf_.child_frame_id = resolved_frame_id_;
  lidar_tf_.transform.translation.x = origin_.x;
  lidar_tf_.transform.translation.y = origin_.y;
  lidar_tf_.transform.translation.z = origin_z_;
  lidar_tf_.transform.rotation.x = q.x();
  lidar_tf_.transform.rotation.y = q.y();
  lidar_tf_.transform.rotation.z = q.z();
  lidar_tf_.transform.rotation.w = q.w();
}

void MockLidar3D::LoadPcd(const std::string &path) {
  std::ifstream f(path, std::ios::binary);
  if (!f) {
    throw YAMLException("MockLidar3D: cannot open pcd_path " + path);
  }

  std::string line, data_mode;
  int fields = 0;
  size_t points = 0;
  while (std::getline(f, line)) {
    std::istringstream ss(line);
    std::string key;
    ss >> key;
    if (key == "FIELDS") {
      std::string v;
      while (ss >> v) fields++;
    } else if (key == "POINTS") {
      ss >> points;
    } else if (key == "DATA") {
      ss >> data_mode;
      break;
    }
  }
  if (fields < 3 || points == 0) {
    throw YAMLException("MockLidar3D: unsupported pcd " + path);
  }

  map_pts_.reserve(points * 3);
  if (data_mode == "binary") {
    std::vector<float> row(fields);
    for (size_t i = 0; i < points; i++) {
      f.read(reinterpret_cast<char *>(row.data()), fields * sizeof(float));
      if (!f) break;
      map_pts_.push_back(row[0]);
      map_pts_.push_back(row[1]);
      map_pts_.push_back(row[2]);
    }
  } else {  // ascii
    double v;
    for (size_t i = 0; i < points && std::getline(f, line); i++) {
      std::istringstream ss(line);
      for (int k = 0; k < fields; k++) {
        if (!(ss >> v)) break;
        if (k < 3) map_pts_.push_back(static_cast<float>(v));
      }
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("MockLidar3D"),
              "MockLidar3D loaded %zu map points from %s", map_pts_.size() / 3,
              path.c_str());
}

void MockLidar3D::BeforePhysicsStep(const Timekeeper &timekeeper) {
  if (pending_scan_.valid() &&
      pending_scan_.wait_for(std::chrono::seconds(0)) ==
          std::future_status::ready) {
    FinishPcdScan();
  }
  if (!update_timer_.CheckUpdate(timekeeper)) {
    return;
  }

  if (broadcast_tf_) {
    lidar_tf_.header.stamp = timekeeper.GetSimTime();
    tf_broadcaster_->sendTransform(lidar_tf_);
  }

  if (cloud_publisher_->get_subscription_count() == 0) {
    return;
  }

  if (!map_pts_.empty()) {
    // A slow scan skips the next capture instead of queuing work or blocking
    // physics.
    if (!pending_scan_.valid()) SamplePcdMode(timekeeper);
  } else {
    SampleExtrusionMode(timekeeper);
  }
}

void MockLidar3D::SamplePcdMode(const Timekeeper &timekeeper) {
  const b2Transform &t = body_->GetPhysicsBody()->GetTransform();
  double sx = t.p.x + t.q.c * origin_.x - t.q.s * origin_.y;
  double sy = t.p.y + t.q.s * origin_.x + t.q.c * origin_.y;
  double yaw = t.q.GetAngle() + origin_.theta;
  double c = cos(yaw), s = sin(yaw);
  b2Vec2 sensor_pos(sx, sy);

  const size_t nch = elevations_.size();
  const size_t nbins = static_cast<size_t>(num_rays_);
  const float max_r2 = static_cast<float>(max_range_ * max_range_);

  scan_stamp_ = timekeeper.GetSimTime();
  dynamic_returns_ = PcdReturns(nbins * nch);
  auto &best_r2 = dynamic_returns_.range_squared;
  auto &best_pt = dynamic_returns_.points;

  // Snapshot dynamic geometry at the capture pose on the physics thread.
  // Worker threads never access Box2D, bodies, or plugin-owned pose state.
  for (size_t i = 0; i < nbins; i++) {
    double wx = c * ray_cos_[i] - s * ray_sin_[i];
    double wy = s * ray_cos_[i] + c * ray_sin_[i];
    b2Vec2 ray_end(sx + max_range_ * wx, sy + max_range_ * wy);

    Lidar3DRayCallback cb(this, true);
    GetModel()->GetPhysicsWorld()->RayCast(&cb, sensor_pos, ray_end);
    if (!cb.did_hit_ || cb.hit_body_ == nullptr) continue;

    double r = cb.closest_fraction_ * max_range_;
    if (r < min_range_) continue;

    double body_h = cb.hit_body_->extrude_height_ > 0.0
                        ? cb.hit_body_->extrude_height_
                        : default_object_height_;
    double z_low = cb.hit_body_->elevation_ - origin_z_;
    double z_high = cb.hit_body_->elevation_ + body_h - origin_z_;

    for (size_t k = 0; k < nch; k++) {
      double z_at = r * std::tan(elevations_[k]);
      if (z_at < z_low || z_at > z_high) continue;
      float r2 = static_cast<float>(r * r + z_at * z_at);
      if (r2 > max_r2) continue;
      size_t idx = i * nch + k;
      if (r2 < best_r2[idx]) {
        best_r2[idx] = r2;
        best_pt[idx] = {static_cast<float>(r * ray_cos_[i]),
                        static_cast<float>(r * ray_sin_[i]),
                        static_cast<float>(z_at)};
      }
    }
  }

  pending_scan_ = std::async(std::launch::async, [
    sampler = pcd_sampler_.get(), sx, sy, sz = origin_z_, yaw
  ]()->PcdReturns { return sampler->Sample(sx, sy, sz, yaw); });
}

void MockLidar3D::FinishPcdScan() {
  auto returns = pending_scan_.get();
  auto &best_r2 = returns.range_squared;
  auto &best_pt = returns.points;
  for (size_t idx = 0; idx < best_r2.size(); ++idx) {
    if (dynamic_returns_.range_squared[idx] < best_r2[idx]) {
      best_r2[idx] = dynamic_returns_.range_squared[idx];
      best_pt[idx] = dynamic_returns_.points[idx];
    }
  }

  // Merge the capture-time snapshots and add fresh radial range noise.
  std::vector<std::array<float, 3>> pts;
  pts.reserve(best_r2.size());
  for (size_t idx = 0; idx < best_r2.size(); idx++) {
    if (best_r2[idx] == std::numeric_limits<float>::max()) continue;
    std::array<float, 3> p = best_pt[idx];
    double r = std::sqrt(best_r2[idx]);
    double scale = (r + range_noise_gen_(rng_)) / r;
    p[0] *= scale;
    p[1] *= scale;
    p[2] *= scale;
    pts.push_back(p);
  }

  PublishCloud(pts, scan_stamp_);
}

void MockLidar3D::SampleExtrusionMode(const Timekeeper &timekeeper) {
  const b2Transform &t = body_->GetPhysicsBody()->GetTransform();
  double sx = t.p.x + t.q.c * origin_.x - t.q.s * origin_.y;
  double sy = t.p.y + t.q.s * origin_.x + t.q.c * origin_.y;
  double yaw = t.q.GetAngle() + origin_.theta;
  double c = cos(yaw), s = sin(yaw);
  b2Vec2 sensor_pos(sx, sy);

  std::vector<std::array<float, 3>> pts;
  for (int i = 0; i < num_rays_; i++) {
    double wx = c * ray_cos_[i] - s * ray_sin_[i];
    double wy = s * ray_cos_[i] + c * ray_sin_[i];
    b2Vec2 ray_end(sx + max_range_ * wx, sy + max_range_ * wy);

    Lidar3DRayCallback cb(this, false);
    GetModel()->GetPhysicsWorld()->RayCast(&cb, sensor_pos, ray_end);
    if (!cb.did_hit_) continue;

    double range = cb.closest_fraction_ * max_range_ + range_noise_gen_(rng_);
    if (range < min_range_ || range > max_range_) continue;
    float px = static_cast<float>(range * ray_cos_[i]);
    float py = static_cast<float>(range * ray_sin_[i]);
    for (double h : heights_) {
      pts.push_back({px, py, static_cast<float>(h)});
    }
  }

  if (ceiling_height_ > 0.0) {
    const int rings = static_cast<int>(
        std::floor((ceiling_max_range_ + 1e-9) / ceiling_ring_step_));
    for (int ring = 0; ring < rings; ++ring) {
      const double r = (ring + 1) * ceiling_ring_step_;
      for (int a = 0; a < ceiling_azimuths_; a++) {
        double angle = a * (2.0 * M_PI / ceiling_azimuths_);
        pts.push_back({static_cast<float>(r * cos(angle)),
                       static_cast<float>(r * sin(angle)),
                       static_cast<float>(ceiling_height_)});
      }
    }
  }

  PublishCloud(pts, timekeeper.GetSimTime());
}

void MockLidar3D::PublishCloud(const std::vector<std::array<float, 3>> &pts,
                               const rclcpp::Time &stamp) {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = resolved_frame_id_;
  cloud.header.stamp = stamp;
  cloud.height = 1;
  cloud.is_dense = true;

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(
      4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
      sensor_msgs::msg::PointField::FLOAT32, "z", 1,
      sensor_msgs::msg::PointField::FLOAT32, "intensity", 1,
      sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(pts.size());

  sensor_msgs::PointCloud2Iterator<float> it_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> it_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> it_z(cloud, "z");
  sensor_msgs::PointCloud2Iterator<float> it_i(cloud, "intensity");
  for (const auto &p : pts) {
    *it_x = p[0];
    *it_y = p[1];
    *it_z = p[2];
    *it_i = 100.0f;
    ++it_x;
    ++it_y;
    ++it_z;
    ++it_i;
  }

  cloud.width = pts.size();
  cloud_publisher_->publish(cloud);
}

float Lidar3DRayCallback::ReportFixture(b2Fixture *fixture, const b2Vec2 &point,
                                        const b2Vec2 &normal, float fraction) {
  // sensors are not real obstacles
  if (fixture->IsSensor()) return -1.0f;

  // only fixtures on the configured layers reflect the rays
  if (!(fixture->GetFilterData().categoryBits & parent_->layers_bits_)) {
    return -1.0f;
  }

  Body *body = static_cast<Body *>(fixture->GetBody()->GetUserData());
  if (!body) return -1.0f;

  Entity *entity = body->GetEntity();
  // never hit the robot carrying the lidar
  if (entity == parent_->GetModel()) return -1.0f;
  // in pcd mode only dynamic bodies count; the static world is in the pcd
  if (models_only_ && entity->Type() != Entity::MODEL) return -1.0f;

  did_hit_ = true;
  closest_fraction_ = fraction;
  hit_body_ = body;
  return fraction;
}

void MockLidar3D::ParseParameters(const YAML::Node &config) {
  YamlReader reader(config);
  std::string body_name = reader.Get<std::string>("body");
  topic_ = reader.Get<std::string>("topic", "/lidar_points");
  frame_id_ = reader.Get<std::string>("frame", GetName());
  broadcast_tf_ = reader.Get<bool>("broadcast_tf", true);
  update_rate_ = reader.Get<double>("update_rate", 10.0);
  origin_ = reader.GetPose("origin", Pose(0, 0, 0));
  origin_z_ = reader.Get<double>("origin_z", 1.0);
  min_range_ = reader.Get<double>("min_range", 0.5);
  max_range_ = reader.Get<double>("max_range", 50.0);
  num_rays_ = reader.Get<int>("num_rays", 720);
  range_noise_std_dev_ = reader.Get<double>("range_noise_std_dev", 0.01);

  // pcd sampling mode
  pcd_path_ = ResolveAssetPath(reader.Get<std::string>("pcd_path", ""));
  sampling_threads_ = reader.Get<int>("sampling_threads", 0);
  // Uniform example beam elevations, in degrees.
  std::vector<double> elevations_deg = reader.GetList<double>(
      "elevations_deg",
      {-15, -12, -9, -6, -3, 0, 3, 6, 9, 12, 15, 18, 21, 24, 27, 30}, 1, -1);
  double elev_tolerance_deg = reader.Get<double>("elev_tolerance_deg", 1.4);
  default_object_height_ = reader.Get<double>("default_object_height", 0.3);

  // legacy extrusion mode
  heights_ = reader.GetList<double>(
      "heights", {2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 6.0}, 1, -1);
  ceiling_height_ = reader.Get<double>("ceiling_height", 0.0);
  ceiling_max_range_ = reader.Get<double>("ceiling_max_range", 10.0);
  ceiling_ring_step_ = reader.Get<double>("ceiling_ring_step", 1.0);
  ceiling_azimuths_ = reader.Get<int>("ceiling_azimuths", 36);

  std::vector<std::string> layers =
      reader.GetList<std::string>("layers", {"all"}, -1, -1);

  reader.EnsureAccessedAllKeys();

  if (sampling_threads_ < 0 || sampling_threads_ > 64) {
    throw YAMLException(
        "sampling_threads must be between 0 (automatic) and 64");
  }
  if (num_rays_ < 8) {
    throw YAMLException("Invalid \"num_rays\", must be >= 8");
  }

  if (max_range_ <= min_range_) {
    throw YAMLException(
        "Invalid \"min_range\"/\"max_range\", must have max_range > "
        "min_range");
  }

  if (ceiling_height_ > 0.0 &&
      (!std::isfinite(ceiling_height_) || !std::isfinite(ceiling_max_range_) ||
       ceiling_max_range_ <= 0.0 || !std::isfinite(ceiling_ring_step_) ||
       ceiling_ring_step_ <= 0.0 || ceiling_azimuths_ <= 0 ||
       (ceiling_max_range_ + 1e-9) / ceiling_ring_step_ >
           std::numeric_limits<int>::max())) {
    throw YAMLException(
        "Ceiling sampling requires finite positive dimensions, "
        "positive azimuths and a representable ring count");
  }

  elevations_.clear();
  for (double e : elevations_deg) {
    elevations_.push_back(e * M_PI / 180.0);
  }
  elev_tolerance_ = elev_tolerance_deg * M_PI / 180.0;

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

  std::random_device rd;
  rng_ = std::default_random_engine(rd());
  range_noise_gen_ =
      std::normal_distribution<double>(0.0, range_noise_std_dev_);

  RCLCPP_INFO(
      rclcpp::get_logger("MockLidar3D"),
      "MockLidar3D %s params: topic(%s) body(%s) origin(%f,%f,%f) "
      "origin_z(%f) frame_id(%s) update_rate(%f) range(%f~%f) num_rays(%d) "
      "mode(%s) pcd(%s) channels(%zu) range_noise_std_dev(%f) layers(0x%u)",
      GetName().c_str(), topic_.c_str(), body_name.c_str(), origin_.x,
      origin_.y, origin_.theta, origin_z_, frame_id_.c_str(), update_rate_,
      min_range_, max_range_, num_rays_,
      pcd_path_.empty() ? "extrusion" : "pcd_sampling", pcd_path_.c_str(),
      elevations_.size(), range_noise_std_dev_, layers_bits_);
}
};  // namespace flatland_plugins

PLUGINLIB_EXPORT_CLASS(flatland_plugins::MockLidar3D,
                       flatland_server::ModelPlugin)
