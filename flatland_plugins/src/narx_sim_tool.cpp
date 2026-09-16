// Copyright (c) 2026, Levent Soysal (Worthle).
// SPDX-License-Identifier: BSD-3-Clause
// Full license notices: LICENSE.

#include <flatland_plugins/narx_model.h>

#include <cstdio>
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

static std::string Trim(const std::string &value) {
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) return "";
  return value.substr(first, value.find_last_not_of(" \t\r\n") - first + 1);
}

int main(int argc, char **argv) try {
  if (argc != 4) {
    std::cerr << "usage: narx_sim_tool <model.yaml> <dataset.csv> "
                 "<n_seed_rows>\n";
    return 2;
  }
  const std::string model_path = argv[1];
  const std::string csv_path = argv[2];
  const int n_seed = std::stoi(argv[3]);
  if (n_seed < 0) throw std::runtime_error("n_seed_rows must be nonnegative");

  flatland_plugins::NarxCoupledModel model;
  try {
    model.Load(model_path);
  } catch (const std::exception &e) {
    std::cerr << e.what() << "\n";
    return 1;
  }

  std::ifstream csv(csv_path);
  if (!csv) {
    std::cerr << "cannot open " << csv_path << "\n";
    return 1;
  }

  // header -> column indexes
  std::string line;
  if (!std::getline(csv, line)) {
    std::cerr << "empty csv\n";
    return 1;
  }
  std::map<std::string, int> col;
  {
    std::stringstream ss(line);
    std::string name;
    int i = 0;
    while (std::getline(ss, name, ',')) {
      name = Trim(name);
      if (name.empty() || col.count(name)) {
        throw std::runtime_error("csv header has an empty or duplicate column");
      }
      col[name] = i++;
    }
  }
  const std::vector<std::string> &cmd_names = model.CommandNames();
  const std::vector<std::string> out_names = model.OutputNames();
  for (const auto &n : cmd_names) {
    if (!col.count(n)) {
      std::cerr << "csv missing command column \"" << n << "\"\n";
      return 1;
    }
  }
  if (n_seed > 0) {
    for (const auto &n : out_names) {
      if (!col.count(n)) {
        std::cerr << "csv missing output column \"" << n
                  << "\" needed for seeding\n";
        return 1;
      }
    }
  }

  // print output header
  for (size_t i = 0; i < out_names.size(); ++i) {
    std::cout << (i ? "," : "") << out_names[i];
  }
  std::cout << "\n";

  int row = 0;
  while (std::getline(csv, line)) {
    if (Trim(line).empty()) continue;
    std::vector<double> fields;
    std::stringstream ss(line);
    std::string cell;
    while (std::getline(ss, cell, ',')) {
      cell = Trim(cell);
      size_t used = 0;
      const double value = std::stod(cell, &used);
      if (used != cell.size() || !std::isfinite(value)) {
        throw std::runtime_error("csv contains a non-finite or invalid number");
      }
      fields.push_back(value);
    }
    if (fields.size() != col.size()) {
      throw std::runtime_error("csv row " + std::to_string(row + 2) +
                               " has a different column count from the header");
    }

    std::vector<double> commands;
    for (const auto &n : cmd_names) commands.push_back(fields[col[n]]);

    std::vector<double> outputs;
    if (row < n_seed) {
      for (const auto &n : out_names) outputs.push_back(fields[col[n]]);
      model.SeedStep(commands, outputs);
    } else {
      outputs = model.Step(commands);
    }
    for (size_t i = 0; i < outputs.size(); ++i) {
      std::printf("%s%.17g", i ? "," : "", outputs[i]);
    }
    std::printf("\n");
    ++row;
  }
  return 0;
} catch (const std::exception &e) {
  std::cerr << e.what() << "\n";
  return 1;
}
