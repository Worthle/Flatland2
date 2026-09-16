// Copyright (c) 2026, Levent Soysal (Worthle).
// SPDX-License-Identifier: BSD-3-Clause
// Full license notices: LICENSE.

#pragma once

#include <Box2D/Box2D.h>
#include <flatland_server/model_plugin.h>
#include <random>
#include <string>
#include <vector>

namespace flatland_plugins {

class TerrainModifier : public flatland_server::ModelPlugin {
 public:
  void OnInitialize(const YAML::Node& config) override;
  void BeforePhysicsStep(const flatland_server::Timekeeper& timekeeper) override;

 private:
  struct SlipParams {
    bool enable = false;
    double s_max_v = 0.0;
    double s_max_w = 0.0;
    double p_at_max = 0.01;
    double hold_time = 0.4;
    double drift_max = 0.0;
  };

  struct RubbleParams {
    bool enable = false;
    double amp_xy = 0.0;
    double amp_yaw = 0.0;
    double f_min = 8.0;
    double f_max = 18.0;
    double speed_gain = 0.0;
  };

  struct SlopeParams {
  bool enable = false;
  double angle_deg = 0.0;
  double downhill_heading_deg = 0.0;
  double rolling_damp = 0.0;
  double v_cap = 0.0;
  double side_slip_gain = 0.0;

  b2Vec2 downhill_unit = b2Vec2(1.0f, 0.0f);
  };


  struct Patch {
    std::string name;
    double xmin = 0.0, xmax = 0.0, ymin = 0.0, ymax = 0.0;

    SlipParams slip;
    RubbleParams rubble;
    SlopeParams slope;

    double slip_v = 0.0;
    double slip_w = 0.0;
    double drift = 0.0;
    double slip_next_update_t = 0.0;

    std::vector<double> vib_f;
    std::vector<double> vib_phi_xy;
    std::vector<double> vib_phi_yaw;
  };

  flatland_server::Body* body_ = nullptr;
  b2Body* b2body_ = nullptr;

  bool enable_ = true;
  bool apply_only_if_moving_ = true;
  double min_speed_ = 1e-3;

  double t_ = 0.0;

  std::mt19937 rng_;
  std::normal_distribution<double> n01_{0.0, 1.0};
  std::uniform_real_distribution<double> uni01_{0.0, 1.0};

  std::vector<Patch> patches_;

  static double Clamp(double x, double lo, double hi);
  static double NormInv(double p);

  bool InPatch(const Patch& p, const b2Vec2& pos) const;

  void ConfigurePatch(Patch& p, const YAML::Node& node);
  void InitRubbleWaves(Patch& p, int n_waves);

  void UpdateSlipState(Patch& p, double dt);

  void ApplyPatch(const Patch& p, double dt);
};

}  // namespace flatland_plugins
