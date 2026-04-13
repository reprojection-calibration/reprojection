#pragma once

#include <filesystem>
#include <optional>
#include <string>

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

// Adopted from https://stackoverflow.com/questions/865668/parsing-command-line-arguments-in-c
std::optional<std::string> GetCommandOption(char const* const* const begin, char const* const* const end,
                                            std::string const& option);

std::optional<toml::table> LoadAndValidateConfig(fs::path const& config_path);

std::optional<SqlitePtr> Open(fs::path const& workspace_dir, fs::path const& data_path);

}  // namespace reprojection::application
