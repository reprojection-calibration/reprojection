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

struct Sensors {
    std::vector<std::string> camera_names;
    std::optional<std::string> imu_name;
};

std::optional<AppArgs> ParseArgs(int const argc, char const* const argv[]);

Sensors ParseSensors(toml::table const& cfg_table);

void Calibrate(toml::table const& cfg_table, ImageInputs const& image_inputs, std::optional<ImuInput> const& imu_input,
               SqlitePtr db);

}  // namespace reprojection::application
