#pragma once

#include <filesystem>

#include <toml++/toml.hpp>

namespace reprojection::config {

namespace fs = std::filesystem;

toml::table LoadConfigFile(fs::path const& cfg_path);

}  // namespace reprojection::config