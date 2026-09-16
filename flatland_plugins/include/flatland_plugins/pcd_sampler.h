// Copyright (c) 2026, Levent Soysal (Worthle).
// SPDX-License-Identifier: BSD-3-Clause
// Full license notices: LICENSE.

#ifndef FLATLAND_PLUGINS_PCD_SAMPLER_H
#define FLATLAND_PLUGINS_PCD_SAMPLER_H

#include <thirdparty/ThreadPool.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <future>
#include <limits>
#include <memory>
#include <vector>

namespace flatland_plugins {

struct PcdReturns {
  std::vector<float> range_squared;
  std::vector<std::array<float, 3>> points;
  explicit PcdReturns(size_t bins)
      : range_squared(bins, std::numeric_limits<float>::max()), points(bins) {}
};

class PcdSampler {
 public:
  PcdSampler(const std::vector<float>& map, std::vector<double> elevations,
             double tolerance, size_t bins, double min_range, double max_range,
             size_t threads)
      : map_(map), elevations_(std::move(elevations)), tolerance_(tolerance),
        bins_(bins), min_r2_(min_range * min_range), max_r2_(max_range * max_range),
        result_(bins * elevations_.size()) {
    const size_t count = std::max<size_t>(1, std::min(threads, map.size() / (3 * 4096)));
    for (size_t i = 0; i < count; ++i) chunks_.emplace_back(bins * elevations_.size());
    if (count > 1) pool_ = std::make_unique<ThreadPool>(count);
  }

  const PcdReturns& Sample(double x, double y, double z, double yaw) {
    const std::array<double, 4> pose{x, y, z, yaw};
    // Only the immutable map projection is cached. The caller still samples
    // moving bodies and adds new measurement noise on every sensor tick.
    if (cached_ && pose == cached_pose_) return result_;
    const double c = std::cos(yaw), s = std::sin(yaw);
    std::vector<std::future<void>> jobs;
    for (size_t chunk = 0; chunk < chunks_.size(); ++chunk) {
      auto project = [&, chunk, x, y, z, c, s]() {
        Project(chunk, x, y, z, c, s);
      };
      if (pool_) jobs.push_back(pool_->enqueue(project));
      else project();
    }
    for (auto& job : jobs) job.get();
    result_ = chunks_[0];
    for (size_t chunk = 1; chunk < chunks_.size(); ++chunk) {
      for (size_t bin = 0; bin < result_.range_squared.size(); ++bin) {
        if (chunks_[chunk].range_squared[bin] < result_.range_squared[bin]) {
          result_.range_squared[bin] = chunks_[chunk].range_squared[bin];
          result_.points[bin] = chunks_[chunk].points[bin];
        }
      }
    }
    cached_pose_ = pose;
    cached_ = true;
    return result_;
  }

 private:
  void Project(size_t chunk, double sx, double sy, double sz, double c, double s) {
    auto& output = chunks_[chunk];
    std::fill(output.range_squared.begin(), output.range_squared.end(),
              std::numeric_limits<float>::max());
    const size_t count = map_.size() / 3;
    const size_t first = count * chunk / chunks_.size();
    const size_t last = count * (chunk + 1) / chunks_.size();
    for (size_t point = first; point < last; ++point) {
      const size_t i = point * 3;
      const double dx = map_[i] - sx, dy = map_[i + 1] - sy, dz = map_[i + 2] - sz;
      const float r2 = static_cast<float>(dx * dx + dy * dy + dz * dz);
      if (!std::isfinite(r2) || r2 > max_r2_ || r2 < min_r2_) continue;
      const double xs = c * dx + s * dy, ys = -s * dx + c * dy;
      const double d2d = std::sqrt(xs * xs + ys * ys);
      if (d2d < 1e-6) continue;
      const double elev = std::atan2(dz, d2d);
      size_t channel = 0;
      double best_diff = std::numeric_limits<double>::max();
      for (size_t k = 0; k < elevations_.size(); ++k) {
        const double diff = std::fabs(elev - elevations_[k]);
        if (diff < best_diff) {
          best_diff = diff;
          channel = k;
        }
      }
      if (best_diff > tolerance_) continue;
      const double az = std::atan2(ys, xs);
      const size_t bin = std::min(bins_ - 1,
          static_cast<size_t>((az + M_PI) / (2.0 * M_PI) * bins_));
      const size_t index = bin * elevations_.size() + channel;
      if (r2 < output.range_squared[index]) {
        output.range_squared[index] = r2;
        output.points[index] = {static_cast<float>(xs), static_cast<float>(ys),
                                static_cast<float>(dz)};
      }
    }
  }

  const std::vector<float>& map_;
  const std::vector<double> elevations_;
  const double tolerance_;
  const size_t bins_;
  const float min_r2_, max_r2_;
  PcdReturns result_;
  bool cached_ = false;
  std::array<double, 4> cached_pose_{};
  std::vector<PcdReturns> chunks_;
  std::unique_ptr<ThreadPool> pool_;
};

}  // namespace flatland_plugins
#endif
