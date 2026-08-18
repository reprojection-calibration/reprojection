#pragma once

#include <filesystem>
#include <optional>

#include <toml++/toml.hpp>

#include "types/io.hpp"

namespace reprojection::application {

namespace fs = std::filesystem;

struct PathConfig {
    fs::path config_path;
    fs::path data_path;
    fs::path workspace_dir;
};

std::optional<PathConfig> ParseCommandLineInput(int const argc, char const* const argv[]);

std::optional<toml::table> LoadConfig(fs::path const& cfg_path);

std::optional<SqlitePtr> Open(fs::path const& workspace_dir, fs::path const& data_path);

}  // namespace reprojection::application