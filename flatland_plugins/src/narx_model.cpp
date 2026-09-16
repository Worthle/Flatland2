// Copyright (c) 2026, Levent Soysal (Worthle).
// SPDX-License-Identifier: BSD-3-Clause
// Full license notices: LICENSE.

#include <flatland_plugins/narx_model.h>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <stdexcept>

namespace flatland_plugins {

namespace {
[[noreturn]] void Fail(const std::string &path, const std::string &what) {
  throw std::runtime_error("NARX model file \"" + path + "\": " + what);
}
}  // namespace

void NarxCoupledModel::Load(const std::string &path) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const YAML::Exception &e) {
    Fail(path, std::string("cannot load YAML (") + e.what() + ")");
  }

  if (!root["sample_rate_hz"]) {
    Fail(path, "missing top-level \"sample_rate_hz\"");
  }
  sample_rate_hz_ = root["sample_rate_hz"].as<double>();
  if (sample_rate_hz_ <= 0.0) {
    Fail(path, "\"sample_rate_hz\" must be > 0");
  }

  YAML::Node subs = root["sub_models"];
  if (!subs || !subs.IsMap() || subs.size() == 0) {
    Fail(path, "missing or empty \"sub_models\" map");
  }

  sub_models_.clear();
  for (const auto &kv : subs) {
    const std::string key = kv.first.as<std::string>();
    YAML::Node node = kv.second;
    NarxSubModel m;
    m.output_name = node["output_name"]
                        ? node["output_name"].as<std::string>()
                        : key;

    YAML::Node inputs = node["input_names"];
    if (!inputs || !inputs.IsSequence() || inputs.size() == 0) {
      Fail(path, "sub-model \"" + key + "\" has no \"input_names\"");
    }
    for (const auto &n : inputs) {
      m.input_names.push_back(n.as<std::string>());
    }

    YAML::Node terms = node["terms"];
    if (!terms || !terms.IsSequence() || terms.size() == 0) {
      Fail(path, "sub-model \"" + key + "\" has no \"terms\"");
    }
    for (const auto &t : terms) {
      NarxTerm term;
      if (!t["theta"]) {
        Fail(path, "sub-model \"" + key + "\" has a term without \"theta\"");
      }
      term.theta = t["theta"].as<double>();
      YAML::Node factors = t["factors"];
      if (factors && !factors.IsSequence()) {
        Fail(path, "sub-model \"" + key + "\" term \"factors\" not a list");
      }
      if (factors) {
        for (const auto &f : factors) {
          if (!f.IsSequence() || f.size() != 2) {
            Fail(path, "sub-model \"" + key +
                           "\" factor must be a [var, lag] pair");
          }
          const std::string var = f[0].as<std::string>();
          NarxFactor factor;
          factor.lag = f[1].as<int>();
          if (factor.lag < 0) {
            Fail(path, "sub-model \"" + key + "\" factor lag < 0");
          }
          if (var == "y") {
            factor.var = -1;
            if (factor.lag < 1) {
              Fail(path, "sub-model \"" + key +
                             "\" own-output factor needs lag >= 1");
            }
          } else if (var.size() >= 2 && var[0] == 'x') {
            int idx = 0;
            try {
              idx = std::stoi(var.substr(1));
            } catch (const std::exception &) {
              Fail(path, "sub-model \"" + key + "\" bad factor var \"" + var +
                             "\"");
            }
            if (idx < 1 || idx > static_cast<int>(m.input_names.size())) {
              Fail(path, "sub-model \"" + key + "\" factor var \"" + var +
                             "\" out of input range");
            }
            factor.var = idx - 1;
          } else {
            Fail(path, "sub-model \"" + key + "\" bad factor var \"" + var +
                           "\" (expected \"y\" or \"xN\")");
          }
          term.factors.push_back(factor);
          m.max_lag = std::max(m.max_lag, factor.lag);
        }
      }
      m.terms.push_back(term);
    }
    sub_models_.push_back(std::move(m));
  }

  // resolve every input: another sub-model's output (cross-coupling, fed by
  // predictions) or an external command signal
  command_names_.clear();
  global_max_lag_ = 1;
  for (NarxSubModel &m : sub_models_) {
    global_max_lag_ = std::max(global_max_lag_, m.max_lag);
    m.input_sources.clear();
    for (size_t i = 0; i < m.input_names.size(); ++i) {
      const std::string &name = m.input_names[i];
      NarxInputSource source;
      source.is_prediction = false;
      source.index = -1;
      for (size_t j = 0; j < sub_models_.size(); ++j) {
        if (sub_models_[j].output_name == name) {
          source.is_prediction = true;
          source.index = static_cast<int>(j);
          break;
        }
      }
      if (!source.is_prediction) {
        auto it = std::find(command_names_.begin(), command_names_.end(), name);
        if (it == command_names_.end()) {
          command_names_.push_back(name);
          source.index = static_cast<int>(command_names_.size()) - 1;
        } else {
          source.index =
              static_cast<int>(std::distance(command_names_.begin(), it));
        }
      }
      m.input_sources.push_back(source);
    }
  }

  // a lag-0 factor on a cross-coupling input would read a value still being
  // computed this step (order-dependent in the reference too) - reject
  for (const NarxSubModel &m : sub_models_) {
    for (const NarxTerm &t : m.terms) {
      for (const NarxFactor &f : t.factors) {
        if (f.var >= 0 && f.lag == 0 &&
            m.input_sources[f.var].is_prediction) {
          Fail(path, "sub-model \"" + m.output_name +
                         "\" uses a cross-coupled input at lag 0");
        }
      }
    }
  }

  Reset();
}

std::vector<std::string> NarxCoupledModel::OutputNames() const {
  std::vector<std::string> names;
  for (const NarxSubModel &m : sub_models_) {
    names.push_back(m.output_name);
  }
  return names;
}

void NarxCoupledModel::Reset() {
  cmd_hist_.assign(command_names_.size(),
                   std::vector<double>(global_max_lag_, 0.0));
  pred_hist_.assign(sub_models_.size(),
                    std::vector<double>(global_max_lag_, 0.0));
}

void NarxCoupledModel::Trim() {
  const size_t keep = static_cast<size_t>(global_max_lag_) + 1;
  const size_t cap = keep * 4;
  for (auto *histories : {&cmd_hist_, &pred_hist_}) {
    for (std::vector<double> &h : *histories) {
      if (h.size() > cap) {
        h.erase(h.begin(), h.end() - keep);
      }
    }
  }
}

std::vector<double> NarxCoupledModel::Step(
    const std::vector<double> &commands) {
  if (commands.size() != command_names_.size()) {
    throw std::runtime_error("NarxCoupledModel::Step: expected " +
                             std::to_string(command_names_.size()) +
                             " commands, got " +
                             std::to_string(commands.size()));
  }

  // commands at sample k become available first (matches simulate_coupled,
  // where the command columns cover the full horizon)
  for (size_t c = 0; c < commands.size(); ++c) {
    cmd_hist_[c].push_back(commands[c]);
  }

  // evaluate every sub-model at k: own output lags and cross-coupling inputs
  // come from the PREDICTION histories (free-run), commands from cmd_hist_
  std::vector<double> outputs(sub_models_.size(), 0.0);
  for (size_t s = 0; s < sub_models_.size(); ++s) {
    const NarxSubModel &m = sub_models_[s];
    double out = 0.0;
    for (const NarxTerm &t : m.terms) {
      double val = t.theta;
      for (const NarxFactor &f : t.factors) {
        if (f.var < 0) {
          // y(k - lag), lag >= 1: last stored prediction is y(k-1)
          const std::vector<double> &h = pred_hist_[s];
          val *= h[h.size() - f.lag];
        } else {
          const NarxInputSource &src = m.input_sources[f.var];
          if (src.is_prediction) {
            const std::vector<double> &h = pred_hist_[src.index];
            val *= h[h.size() - f.lag];
          } else {
            // cmd(k - lag), lag >= 0: last stored command is cmd(k)
            const std::vector<double> &h = cmd_hist_[src.index];
            val *= h[h.size() - 1 - f.lag];
          }
        }
      }
      out += val;
    }
    outputs[s] = out;
  }

  for (size_t s = 0; s < sub_models_.size(); ++s) {
    pred_hist_[s].push_back(outputs[s]);
  }
  Trim();
  return outputs;
}

void NarxCoupledModel::SeedStep(const std::vector<double> &commands,
                                const std::vector<double> &outputs) {
  if (commands.size() != command_names_.size() ||
      outputs.size() != sub_models_.size()) {
    throw std::runtime_error("NarxCoupledModel::SeedStep: size mismatch");
  }
  for (size_t c = 0; c < commands.size(); ++c) {
    cmd_hist_[c].push_back(commands[c]);
  }
  for (size_t s = 0; s < outputs.size(); ++s) {
    pred_hist_[s].push_back(outputs[s]);
  }
  Trim();
}

}  // namespace flatland_plugins
