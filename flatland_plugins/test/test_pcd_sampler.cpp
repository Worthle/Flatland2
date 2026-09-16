// Copyright (c) 2026, Levent Soysal (Worthle).
// SPDX-License-Identifier: BSD-3-Clause

#include <flatland_plugins/pcd_sampler.h>
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

namespace flatland_plugins {

TEST(PcdSampler, SelectsNearestReturnInEachAngularBin) {
  const std::vector<float> map{3, 0, 0, 2, 0, 0, 0, 4, 0};
  PcdSampler sampler(map, {0.0}, 0.01, 4, 0.1, 10, 1);
  const auto &returns = sampler.Sample(0, 0, 0, 0);
  EXPECT_FLOAT_EQ(4.0f, returns.range_squared[2]);
  EXPECT_FLOAT_EQ(16.0f, returns.range_squared[3]);
  EXPECT_EQ((std::array<float, 3>{2, 0, 0}), returns.points[2]);
}

TEST(PcdSampler, FiltersRangeElevationAndNonFinitePoints) {
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const std::vector<float> map{0.2f, 0, 0, 20, 0, 0, 1, 0, 2, nan, 0, 0};
  PcdSampler sampler(map, {0.0}, 0.01, 4, 0.5, 10, 1);
  for (const float distance : sampler.Sample(0, 0, 0, 0).range_squared) {
    EXPECT_EQ(std::numeric_limits<float>::max(), distance);
  }
}

TEST(PcdSampler, UpdatesCachedProjectionWhenSensorPoseChanges) {
  const std::vector<float> map{2, 0, 0};
  PcdSampler sampler(map, {0.0}, 0.01, 4, 0.1, 10, 1);
  EXPECT_FLOAT_EQ(4.0f, sampler.Sample(0, 0, 0, 0).range_squared[2]);
  EXPECT_FLOAT_EQ(1.0f, sampler.Sample(1, 0, 0, 0).range_squared[2]);
  const auto &rotated = sampler.Sample(0, 0, 0, M_PI / 2);
  EXPECT_FLOAT_EQ(4.0f, rotated.range_squared[1]);
  EXPECT_NEAR(0.0f, rotated.points[1][0], 1e-6);
  EXPECT_FLOAT_EQ(-2.0f, rotated.points[1][1]);
}

TEST(PcdSampler, ParallelProjectionMatchesSerialProjection) {
  std::vector<float> map;
  for (int i = 0; i < 10000; ++i) {
    const double angle = 2.0 * M_PI * i / 10000;
    const double radius = 2.0 + i % 5;
    map.insert(map.end(), {static_cast<float>(radius * std::cos(angle)),
                           static_cast<float>(radius * std::sin(angle)), 0.0f});
  }
  PcdSampler serial(map, {0.0}, 0.01, 360, 0.1, 10, 1);
  PcdSampler parallel(map, {0.0}, 0.01, 360, 0.1, 10, 4);
  const auto expected = serial.Sample(0.1, -0.1, 0, 0.2);
  const auto actual = parallel.Sample(0.1, -0.1, 0, 0.2);
  EXPECT_EQ(expected.range_squared, actual.range_squared);
  EXPECT_EQ(expected.points, actual.points);
}

}  // namespace flatland_plugins
