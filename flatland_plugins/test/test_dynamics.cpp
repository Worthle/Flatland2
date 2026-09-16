// Copyright (c) 2026, Levent Soysal (Worthle).
// SPDX-License-Identifier: BSD-3-Clause

#include <flatland_plugins/discrete_oe_model.h>
#include <flatland_plugins/dynamics_limits.h>
#include <flatland_plugins/narx_model.h>
#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>
#include <vector>

namespace flatland_plugins {

TEST(DiscreteOEModel, PassesThroughBeforeConfiguration) {
  DiscreteOEModel model;
  EXPECT_FALSE(model.IsConfigured());
  EXPECT_DOUBLE_EQ(-2.0, model.Step(-2.0));
}

TEST(DiscreteOEModel, AppliesInputDelayAndFeedback) {
  DiscreteOEModel model;
  model.Configure({0.5}, {-0.5}, 1, 0.1);
  EXPECT_TRUE(model.IsConfigured());
  EXPECT_DOUBLE_EQ(0.1, model.GetSampleTime());
  EXPECT_DOUBLE_EQ(0.0, model.Step(1.0));
  EXPECT_DOUBLE_EQ(0.5, model.Step(1.0));
  EXPECT_DOUBLE_EQ(0.75, model.Step(0.0));
  EXPECT_DOUBLE_EQ(0.375, model.Step(0.0));
  model.Reset();
  EXPECT_DOUBLE_EQ(0.0, model.Step(0.0));
}

TEST(DiscreteOEModel, RejectsInvalidConfiguration) {
  DiscreteOEModel model;
  EXPECT_THROW(model.Configure({}, {}, 0, 0.1), std::invalid_argument);
  EXPECT_THROW(model.Configure({1.0}, {}, -1, 0.1), std::invalid_argument);
  EXPECT_THROW(model.Configure({1.0}, {}, 0, 0.0), std::invalid_argument);
  EXPECT_THROW(
      model.Configure({1.0}, {}, 0, std::numeric_limits<double>::quiet_NaN()),
      std::invalid_argument);
}

TEST(DynamicsLimits, LimitsAccelerationAndDecelerationAcrossZero) {
  DynamicsLimits limits;
  limits.acceleration_limit_ = 1.0;
  limits.deceleration_limit_ = 2.0;
  limits.velocity_limit_ = 3.0;
  EXPECT_DOUBLE_EQ(0.5, limits.Limit(0.0, 10.0, 0.5));
  EXPECT_DOUBLE_EQ(0.5, limits.Limit(1.0, 0.0, 0.25));
  EXPECT_DOUBLE_EQ(-0.5, limits.Limit(1.0, -10.0, 1.0));
  EXPECT_DOUBLE_EQ(3.0, limits.Limit(3.0, 10.0, 1.0));
}

TEST(NarxModel, UsesPreviousPredictionsForCoupledOutputs) {
  NarxCoupledModel model;
  model.Load(NARX_FIXTURE);
  EXPECT_DOUBLE_EQ(10.0, model.SampleRateHz());
  EXPECT_EQ((std::vector<std::string>{"command"}), model.CommandNames());
  EXPECT_EQ((std::vector<std::string>{"speed", "response"}),
            model.OutputNames());
  EXPECT_EQ((std::vector<double>{0.0, 0.0}), model.Step({1.0}));
  EXPECT_EQ((std::vector<double>{0.5, 0.0}), model.Step({1.0}));
  EXPECT_EQ((std::vector<double>{0.75, 1.0}), model.Step({0.0}));
  EXPECT_EQ((std::vector<double>{0.375, 1.5}), model.Step({0.0}));
}

TEST(NarxModel, SeedsAndResetsHistory) {
  NarxCoupledModel model;
  model.Load(NARX_FIXTURE);
  model.SeedStep({2.0}, {4.0, 8.0});
  EXPECT_EQ((std::vector<double>{3.0, 8.0}), model.Step({0.0}));
  model.Reset();
  EXPECT_EQ((std::vector<double>{0.0, 0.0}), model.Step({0.0}));
}

TEST(NarxModel, RejectsWrongSignalCounts) {
  NarxCoupledModel model;
  model.Load(NARX_FIXTURE);
  EXPECT_THROW(model.Step({}), std::runtime_error);
  EXPECT_THROW(model.Step({1.0, 2.0}), std::runtime_error);
  EXPECT_THROW(model.SeedStep({1.0}, {1.0}), std::runtime_error);
}

TEST(NarxModel, KeepsLongRunsConsistentWhenTrimmingHistory) {
  NarxCoupledModel model;
  model.Load(NARX_FIXTURE);
  double speed = 0.0;
  double previous_command = 0.0;
  for (int i = 0; i < 1000; ++i) {
    const double command = i < 500 ? 1.0 : -1.0;
    const auto result = model.Step({command});
    ASSERT_EQ(2u, result.size());
    EXPECT_DOUBLE_EQ(2.0 * speed, result[1]);
    speed = 0.5 * speed + 0.5 * previous_command;
    EXPECT_DOUBLE_EQ(speed, result[0]);
    previous_command = command;
  }
}

}  // namespace flatland_plugins
