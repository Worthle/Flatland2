// Copyright (c) 2026, Levent Soysal (Worthle).
// SPDX-License-Identifier: BSD-3-Clause
// Full license notices: LICENSE.

#pragma once

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <stdexcept>
#include <string>

namespace flatland_plugins {

// Resolve installed assets without depending on a workspace location.
inline std::string ResolveAssetPath(const std::string &path) {
  const std::string prefix = "package://";
  if (path.compare(0, prefix.size(), prefix) != 0) return path;
  const auto separator = path.find('/', prefix.size());
  if (separator == std::string::npos || separator == prefix.size() ||
      separator + 1 == path.size()) {
    throw std::invalid_argument(
        "Asset URI must have the form package://package/path");
  }
  return ament_index_cpp::get_package_share_directory(
             path.substr(prefix.size(), separator - prefix.size())) +
         path.substr(separator);
}

}  // namespace flatland_plugins
