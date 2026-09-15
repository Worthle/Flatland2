// Copyright (c) 2026, Levent Soysal (Worthle).
// SPDX-License-Identifier: BSD-3-Clause
// Full license notices: LICENSE and SOURCE_NOTICES.

#ifndef FLATLAND_PLUGINS_NARX_MODEL_H
#define FLATLAND_PLUGINS_NARX_MODEL_H

#include <string>
#include <vector>

namespace flatland_plugins {

/// one product factor of a polynomial term: variable index and lag
struct NarxFactor {
  int var;  ///< -1 = the sub-model's own output 'y', >= 0 = input index
  int lag;  ///< samples of delay (y requires >= 1)
};

/// one polynomial term: theta * prod_j var_j(k - lag_j)
struct NarxTerm {
  double theta;
  std::vector<NarxFactor> factors;  ///< empty = constant term
};

/// where a sub-model input takes its values from during coupled free-run
struct NarxInputSource {
  bool is_prediction;  ///< true: another sub-model's output history
  int index;           ///< sub-model index or command index
};

/// one MISO polynomial NARX sub-model (PolyNARX equivalent)
struct NarxSubModel {
  std::string output_name;
  std::vector<std::string> input_names;
  std::vector<NarxInputSource> input_sources;
  std::vector<NarxTerm> terms;
  int max_lag = 1;
};

/**
 * Coupled free-run simulator of all sub-models in a SystemID NARMAX model
 * file (simulate_coupled equivalent), advancing one sample per Step() call
 * at the model's native sample_rate_hz. Histories initialize to zero (robot
 * at rest); SeedStep() can push known samples without evaluating (used by
 * the offline verification tool to replay recorded initial conditions).
 */
class NarxCoupledModel {
 public:
  /**
   * @brief Load and validate a SystemID NARMAX model YAML
   * @param[in] path path to the model file
   * @throw std::runtime_error with a descriptive message on any file or
   *        format problem
   */
  void Load(const std::string &path);

  /// model native sample rate [Hz]; Step() advances one sample
  double SampleRateHz() const { return sample_rate_hz_; }

  /// external command inputs, in the order Step() expects them
  const std::vector<std::string> &CommandNames() const {
    return command_names_;
  }

  /// sub-model outputs, in the order Step() returns them
  std::vector<std::string> OutputNames() const;

  /**
   * @brief Advance the coupled free-run by one sample
   * @param[in] commands current command values, ordered per CommandNames()
   * @return predicted outputs, ordered per OutputNames()
   */
  std::vector<double> Step(const std::vector<double> &commands);

  /**
   * @brief Push one sample of known commands AND outputs without evaluating
   * (initial-condition replay for offline verification)
   */
  void SeedStep(const std::vector<double> &commands,
                const std::vector<double> &outputs);

  /// zero all histories (robot at rest)
  void Reset();

  const std::vector<NarxSubModel> &SubModels() const { return sub_models_; }
  int GlobalMaxLag() const { return global_max_lag_; }

 private:
  double sample_rate_hz_ = 0.0;
  std::vector<NarxSubModel> sub_models_;
  std::vector<std::string> command_names_;
  int global_max_lag_ = 1;

  /// chronological histories; the newest sample is the last element. Command
  /// histories are one sample "ahead" of prediction histories inside Step()
  /// (commands at k are pushed before the outputs at k are computed).
  std::vector<std::vector<double>> cmd_hist_;   ///< per command
  std::vector<std::vector<double>> pred_hist_;  ///< per sub-model

  void Trim();
};

}  // namespace flatland_plugins

#endif  // FLATLAND_PLUGINS_NARX_MODEL_H
