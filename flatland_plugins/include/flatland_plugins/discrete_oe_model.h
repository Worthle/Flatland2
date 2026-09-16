// Copyright (c) 2026, Levent Soysal (Worthle).
// SPDX-License-Identifier: BSD-3-Clause
// Full license notices: LICENSE.

#ifndef FLATLAND_PLUGINS_DISCRETE_OE_MODEL_H
#define FLATLAND_PLUGINS_DISCRETE_OE_MODEL_H

#include <cmath>
#include <cstddef>
#include <deque>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <vector>

namespace flatland_plugins {

class DiscreteOEModel {
 public:
  DiscreteOEModel() : nk_(0), Ts_(0.0), initialized_(false) {}

  /**
   * @brief Configure the OE model with polynomial coefficients
   * @param B  B polynomial coefficients [b0, b1, ..., bN]
   * @param F  F polynomial coefficients WITHOUT leading 1: [f1, f2, ..., fM]
   * @param nk Input delay in samples
   * @param Ts Sample time in seconds
   */
  void Configure(const std::vector<double>& B, const std::vector<double>& F,
                 int nk, double Ts) {
    if (B.empty() || nk < 0 || !std::isfinite(Ts) || Ts <= 0.0) {
      throw std::invalid_argument(
          "OE model requires coefficients, nonnegative delay and positive "
          "sample time");
    }
    B_ = B;
    F_ = F;
    nk_ = nk;
    Ts_ = Ts;

    // We need to store enough past inputs: nk + nb samples
    // nb = B_.size() (number of B coefficients, so indices 0..nb-1)
    // Total past input slots needed: nk + B_.size()
    size_t u_hist_size = nk_ + B_.size();
    u_history_.assign(u_hist_size, 0.0);

    // Past output slots needed: F_.size() (nf)
    y_history_.assign(F_.size(), 0.0);

    initialized_ = true;

    RCLCPP_INFO(rclcpp::get_logger("flatland"),
                "DiscreteOEModel configured: nb=%zu, nf=%zu, nk=%d, Ts=%.4f",
                B_.size(), F_.size(), nk_, Ts_);
  }

  /**
   * @brief Advance one sample step
   * @param u Current input value u(k)
   * @return y(k) model output
   *
   * Call this once per sample period Ts.
   */
  double Step(double u) {
    if (!initialized_) return u;  // passthrough if not configured

    // Push new input to front (index 0 = current, 1 = one step ago, etc.)
    u_history_.push_front(u);
    if (u_history_.size() > nk_ + B_.size()) {
      u_history_.pop_back();
    }

    // Compute y(k) = sum_i B[i] * u(k - nk - i) - sum_j F[j] * y(k - 1 - j)
    double y = 0.0;

    // B contribution: b[i] * u(k - nk - i) for i = 0..nb-1
    for (size_t i = 0; i < B_.size(); i++) {
      size_t idx = nk_ + i;  // delay + coefficient index
      if (idx < u_history_.size()) {
        y += B_[i] * u_history_[idx];
      }
    }

    // F contribution: -f[j] * y(k - 1 - j) for j = 0..nf-1
    // F_ stores [f1, f2, ...], y_history_[0] = y(k-1), y_history_[1] = y(k-2),
    // etc.
    for (size_t j = 0; j < F_.size(); j++) {
      if (j < y_history_.size()) {
        y -= F_[j] * y_history_[j];
      }
    }

    // Push y(k) into output history
    y_history_.push_front(y);
    if (y_history_.size() > F_.size()) {
      y_history_.pop_back();
    }

    return y;
  }

  /**
   * @brief Reset all internal states to zero
   */
  void Reset() {
    for (auto& v : u_history_) v = 0.0;
    for (auto& v : y_history_) v = 0.0;
  }

  bool IsConfigured() const { return initialized_; }
  double GetSampleTime() const { return Ts_; }

 private:
  std::vector<double> B_;  ///< B polynomial coefficients
  std::vector<double> F_;  ///< F polynomial coefficients (without leading 1)
  int nk_;                 ///< Input delay in samples
  double Ts_;              ///< Sample time (seconds)
  bool initialized_;

  std::deque<double> u_history_;  ///< Past input values
  std::deque<double> y_history_;  ///< Past output values
};

}  // namespace flatland_plugins

#endif  // FLATLAND_PLUGINS_DISCRETE_OE_MODEL_H
