// Copyright (c) 2026, Levent Soysal (Worthle).
// SPDX-License-Identifier: BSD-3-Clause
// Full license notices: LICENSE and SOURCE_NOTICES.

#include <flatland_plugins/terrain_modifier.h>

#include <flatland_server/model_plugin.h>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>
#include <limits>

namespace flatland_plugins {

double TerrainModifier::Clamp(double x, double lo, double hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

double TerrainModifier::NormInv(double p) {
  p = Clamp(p, 1e-12, 1.0 - 1e-12);

  const double a1 = -3.969683028665376e+01;
  const double a2 =  2.209460984245205e+02;
  const double a3 = -2.759285104469687e+02;
  const double a4 =  1.383577518672690e+02;
  const double a5 = -3.066479806614716e+01;
  const double a6 =  2.506628277459239e+00;

  const double b1 = -5.447609879822406e+01;
  const double b2 =  1.615858368580409e+02;
  const double b3 = -1.556989798598866e+02;
  const double b4 =  6.680131188771972e+01;
  const double b5 = -1.328068155288572e+01;

  const double c1 = -7.784894002430293e-03;
  const double c2 = -3.223964580411365e-01;
  const double c3 = -2.400758277161838e+00;
  const double c4 = -2.549732539343734e+00;
  const double c5 =  4.374664141464968e+00;
  const double c6 =  2.938163982698783e+00;

  const double d1 =  7.784695709041462e-03;
  const double d2 =  3.224671290700398e-01;
  const double d3 =  2.445134137142996e+00;
  const double d4 =  3.754408661907416e+00;

  const double plow  = 0.02425;
  const double phigh = 1.0 - plow;

  double q, r;
  if (p < plow) {
    q = std::sqrt(-2.0 * std::log(p));
    return (((((c1*q + c2)*q + c3)*q + c4)*q + c5)*q + c6) /
           ((((d1*q + d2)*q + d3)*q + d4)*q + 1.0);
  }
  if (p > phigh) {
    q = std::sqrt(-2.0 * std::log(1.0 - p));
    return -(((((c1*q + c2)*q + c3)*q + c4)*q + c5)*q + c6) /
            ((((d1*q + d2)*q + d3)*q + d4)*q + 1.0);
  }

  q = p - 0.5;
  r = q*q;
  return (((((a1*r + a2)*r + a3)*r + a4)*r + a5)*r + a6)*q /
         (((((b1*r + b2)*r + b3)*r + b4)*r + b5)*r + 1.0);
}

bool TerrainModifier::InPatch(const Patch& p, const b2Vec2& pos) const {
  return (pos.x >= p.xmin && pos.x <= p.xmax && pos.y >= p.ymin && pos.y <= p.ymax);
}

void TerrainModifier::InitRubbleWaves(Patch& p, int n_waves) {
  p.vib_f.clear();
  p.vib_phi_xy.clear();
  p.vib_phi_yaw.clear();

  if (n_waves <= 0) return;

  std::uniform_real_distribution<double> unif_f(p.rubble.f_min, p.rubble.f_max);
  std::uniform_real_distribution<double> unif_phi(0.0, 2.0 * M_PI);

  p.vib_f.resize(n_waves);
  p.vib_phi_xy.resize(2 * n_waves);
  p.vib_phi_yaw.resize(n_waves);

  for (int i = 0; i < n_waves; ++i) {
    p.vib_f[i] = unif_f(rng_);
    p.vib_phi_xy[2*i + 0] = unif_phi(rng_);
    p.vib_phi_xy[2*i + 1] = unif_phi(rng_);
    p.vib_phi_yaw[i] = unif_phi(rng_);
  }
}

void TerrainModifier::ConfigurePatch(Patch& p, const YAML::Node& node) {
  flatland_server::YamlReader r(node);

  p.name = r.Get<std::string>("name", std::string("patch"));

  auto b = r.GetList<double>("box", {0, 0, 0, 0}, 4, 4);
  p.xmin = std::min(b[0], b[2]);
  p.ymin = std::min(b[1], b[3]);
  p.xmax = std::max(b[0], b[2]);
  p.ymax = std::max(b[1], b[3]);

  p.slip_next_update_t = 0.0;
  p.slip_v = 0.0;
  p.slip_w = 0.0;
  p.drift  = 0.0;

  auto slip_node = r.SubnodeOpt("slip", flatland_server::YamlReader::MAP);
  if (slip_node.Node()) {
    flatland_server::YamlReader s(slip_node.Node());
    p.slip.enable    = s.Get<bool>("enable", true);
    p.slip.s_max_v   = s.Get<double>("s_max_v", 0.0);
    p.slip.s_max_w   = s.Get<double>("s_max_w", 0.0);
    p.slip.p_at_max  = s.Get<double>("p_at_max", 0.01);
    p.slip.hold_time = s.Get<double>("hold_time", 0.4);
    p.slip.drift_max = s.Get<double>("drift_max", 0.0);
    s.EnsureAccessedAllKeys();
  }

  auto rubble_node = r.SubnodeOpt("rubble", flatland_server::YamlReader::MAP);
  if (rubble_node.Node()) {
    flatland_server::YamlReader rb(rubble_node.Node());

    int n_waves = rb.Get<int>("n_waves", 3);

    p.rubble.enable     = rb.Get<bool>("enable", true);
    p.rubble.amp_xy     = rb.Get<double>("amp_xy", 0.0);
    p.rubble.amp_yaw    = rb.Get<double>("amp_yaw", 0.0);
    p.rubble.f_min      = rb.Get<double>("f_min", 8.0);
    p.rubble.f_max      = rb.Get<double>("f_max", 18.0);
    p.rubble.speed_gain = rb.Get<double>("speed_gain", 0.0);

    rb.EnsureAccessedAllKeys();

    if (p.rubble.enable) {
      InitRubbleWaves(p, n_waves);
    }
  }
  auto slope_node = r.SubnodeOpt("slope", flatland_server::YamlReader::MAP);
  if (slope_node.Node()) {
    flatland_server::YamlReader sp(slope_node.Node());
    p.slope.enable               = sp.Get<bool>("enable", true);
    p.slope.angle_deg            = sp.Get<double>("angle_deg", 0.0);
    p.slope.downhill_heading_deg = sp.Get<double>("downhill_heading_deg", 0.0);
    p.slope.rolling_damp         = sp.Get<double>("rolling_damp", 0.0);
    p.slope.v_cap                = sp.Get<double>("v_cap", 0.0);
    p.slope.side_slip_gain       = sp.Get<double>("side_slip_gain", 0.0);
    sp.EnsureAccessedAllKeys();

    double th = p.slope.downhill_heading_deg * M_PI / 180.0;
    p.slope.downhill_unit = b2Vec2((float)std::cos(th), (float)std::sin(th));
  }

  r.EnsureAccessedAllKeys();
}


void TerrainModifier::OnInitialize(const YAML::Node& config) {
  flatland_server::YamlReader reader(config);

  enable_ = reader.Get<bool>("enable", true);

  std::string body_name = reader.Get<std::string>("body");
  apply_only_if_moving_ = reader.Get<bool>("apply_only_if_moving", true);
  min_speed_ = reader.Get<double>("min_speed", 1e-3);

  int seed = reader.Get<int>("seed", 0);
  if (seed == 0) {
    std::random_device rd;
    rng_.seed(rd());
  } else {
    rng_.seed(static_cast<uint32_t>(seed));
  }

  body_ = GetModel()->GetBody(body_name);
  if (body_ == nullptr) {
    throw flatland_server::YAMLException("Body with name " + flatland_server::Q(body_name) + " does not exist");
  }
  b2body_ = body_->physics_body_;

  patches_.clear();
    auto patches_node = reader.SubnodeOpt("patches", flatland_server::YamlReader::LIST);
    if (patches_node.Node() && patches_node.Node().IsSequence()) {
    for (const auto& n : patches_node.Node()) {
        Patch p;
        ConfigurePatch(p, n);
        patches_.push_back(p);
    }
    }

reader.EnsureAccessedAllKeys();


  reader.EnsureAccessedAllKeys();

  RCLCPP_INFO(rclcpp::get_logger("TerrainModifier"),
                 "TerrainModifier initialized: body=%s patches=%zu enable=%d",
                 body_name.c_str(), patches_.size(), (int)enable_);
}

void TerrainModifier::UpdateSlipState(Patch& p, double dt) {
  if (!p.slip.enable) return;

  if (p.slip.hold_time <= 0.0) p.slip.hold_time = dt;

  if (t_ < p.slip_next_update_t) return;

  const double pmax = Clamp(p.slip.p_at_max, 1e-9, 0.999999);
  const double z = NormInv(1.0 - pmax * 0.5);

  auto sample_trunc_abs_gauss = [&](double smax) -> double {
    if (smax <= 0.0) return 0.0;
    const double sigma = smax / std::max(1e-9, z);
    double s = std::fabs(sigma * n01_(rng_));
    if (s > smax) s = smax;
    return s;
  };

  p.slip_v = sample_trunc_abs_gauss(p.slip.s_max_v);
  p.slip_w = sample_trunc_abs_gauss(p.slip.s_max_w);

  if (p.slip.drift_max > 0.0) {
    double d = n01_(rng_) * (p.slip.drift_max / 3.0);
    p.drift = Clamp(d, -p.slip.drift_max, p.slip.drift_max);
  } else {
    p.drift = 0.0;
  }

  p.slip_next_update_t = t_ + p.slip.hold_time;
}

void TerrainModifier::ApplyPatch(const Patch& p, double dt) {
  b2Vec2 v_world = b2body_->GetLinearVelocity();
  float w = b2body_->GetAngularVelocity();

  b2Vec2 v_local = b2body_->GetLocalVector(v_world);

  auto speed_local = [&]() -> double {
    return std::sqrt((double)v_local.x * (double)v_local.x + (double)v_local.y * (double)v_local.y);
  };

  double sp0 = speed_local();

  // -----------------------
  // SLOPE: always applied
  // -----------------------
  if (p.slope.enable && dt > 0.0) {
    const double ang = p.slope.angle_deg * M_PI / 180.0;
    const double g = 9.81;
    const double a = g * std::sin(ang);

    b2Vec2 fwd_world = b2body_->GetWorldVector(b2Vec2(1.0f, 0.0f));
    double fn = std::sqrt((double)fwd_world.x * (double)fwd_world.x + (double)fwd_world.y * (double)fwd_world.y);
    if (fn > 1e-9) {
      fwd_world.x /= (float)fn;
      fwd_world.y /= (float)fn;
    }

    double c = (double)fwd_world.x * (double)p.slope.downhill_unit.x +
               (double)fwd_world.y * (double)p.slope.downhill_unit.y;

    double vxf = (double)v_local.x;

    vxf += (a * c) * dt;

    if (p.slope.rolling_damp > 0.0) {
      double k = p.slope.rolling_damp;
      vxf *= std::exp(-k * dt);
    }

    if (p.slope.v_cap > 0.0) {
      vxf = Clamp(vxf, -p.slope.v_cap, p.slope.v_cap);
    }

    v_local.x = (float)vxf;
    v_world = b2body_->GetWorldVector(v_local);
    b2body_->SetLinearVelocity(v_world);
    b2body_->SetAwake(true);
  }

  // recompute after slope
  v_world = b2body_->GetLinearVelocity();
  w = b2body_->GetAngularVelocity();
  v_local = b2body_->GetLocalVector(v_world);

  double sp = speed_local();
  bool moving_now = (sp >= min_speed_) || (std::fabs((double)w) >= min_speed_);

  // -----------------------
  // SLIP + RUBBLE: only if moving (based on actual body velocity)
  // -----------------------
  if (moving_now) {
    double slip_scale = 1.0;

    if (p.slope.enable && p.slope.side_slip_gain > 0.0) {
      b2Vec2 fwd_world = b2body_->GetWorldVector(b2Vec2(1.0f, 0.0f));
      double fn = std::sqrt((double)fwd_world.x * (double)fwd_world.x + (double)fwd_world.y * (double)fwd_world.y);
      if (fn > 1e-9) {
        fwd_world.x /= (float)fn;
        fwd_world.y /= (float)fn;
      }

      double c = (double)fwd_world.x * (double)p.slope.downhill_unit.x +
                 (double)fwd_world.y * (double)p.slope.downhill_unit.y;
      c = Clamp(c, -1.0, 1.0);
      double s = std::sqrt(std::max(0.0, 1.0 - c * c));  // 0 when aligned, 1 when sideways
      slip_scale = 1.0 + p.slope.side_slip_gain * s;
    }

    if (p.slip.enable) {
      double sv = Clamp(p.slip_v * slip_scale, 0.0, 0.999);
      double sw = Clamp(p.slip_w * slip_scale, 0.0, 0.999);

      v_local.x *= (float)(1.0 - sv);
      v_local.y += (float)(p.drift * slip_scale);
      w *= (float)(1.0 - sw);
    }

    if (p.rubble.enable && !p.vib_f.empty()) {
      double Axy  = p.rubble.amp_xy  + p.rubble.speed_gain * sp;
      double Ayaw = p.rubble.amp_yaw + p.rubble.speed_gain * std::fabs((double)w);

      double jx = 0.0, jy = 0.0, jw = 0.0;
      const int N = (int)p.vib_f.size();

      for (int i = 0; i < N; ++i) {
        double wi = 2.0 * M_PI * p.vib_f[i];
        jx += std::sin(wi * t_ + p.vib_phi_xy[2 * i + 0]);
        jy += std::sin(wi * t_ + p.vib_phi_xy[2 * i + 1]);
        jw += std::sin(wi * t_ + p.vib_phi_yaw[i]);
      }

      double invN = 1.0 / std::max(1, N);
      jx *= invN; jy *= invN; jw *= invN;

      v_local.x += (float)(Axy * jx);
      v_local.y += (float)(Axy * jy);
      w += (float)(Ayaw * jw);
    }
  }

  b2Vec2 v_world_mod = b2body_->GetWorldVector(v_local);
  b2body_->SetLinearVelocity(v_world_mod);
  b2body_->SetAngularVelocity(w);
  b2body_->SetAwake(true);
}



void TerrainModifier::BeforePhysicsStep(const flatland_server::Timekeeper& timekeeper) {
  if (!enable_) return;

  const double dt = timekeeper.GetStepSize();
  t_ += dt;

  if (patches_.empty()) return;

  b2Vec2 pos = b2body_->GetPosition();

  int idx = -1;
  for (int i = 0; i < (int)patches_.size(); ++i) {
    if (InPatch(patches_[i], pos)) {
      idx = i;
      break;
    }
  }
  if (idx < 0) return;

  Patch& p = patches_[idx];
  UpdateSlipState(p, dt);
  ApplyPatch(p, dt);
}

}  // namespace flatland_plugins

PLUGINLIB_EXPORT_CLASS(flatland_plugins::TerrainModifier, flatland_server::ModelPlugin)
