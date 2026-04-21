#pragma once

#include <filesystem>
#include <optional>

#include <toml++/toml.hpp>

#include "types/io.hpp"

namespace reprojection::application {

namespace fs = std::filesystem;

struct AppArgs {
    fs::path data_path;
    toml::table config;
    SqlitePtr db;
};

std::optional<AppArgs> ParseArgs(int const argc, char const* const argv[]);

// TODO(Jack): How should we pass the ImageSource?
void Calibrate(toml::table const& config, ImageSource image_source, std::string const& image_source_signature,
               SqlitePtr const db);

}  // namespace reprojection::application
