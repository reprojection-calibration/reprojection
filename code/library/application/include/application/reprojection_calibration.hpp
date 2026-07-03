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
    std::string camera_sensor;
    std::optional<std::string> imu_sensor;
};

struct ImageInput {
    ImageSampler source;
    std::string signature;
};

// TODO(Jack): Once we get the app running in unit testing with test imu data we can remove this coverage exclusion!
struct ImuInput {  // LCOV_EXCL_LINE
    ImuSampler source;
    std::string signature;
};

std::optional<AppArgs> ParseArgs(int const argc, char const* const argv[]);

Sensors ParseSensors(toml::table const& cfg_table);

// TODO(Jack): How should we pass the ImageSourceSignature?
void Calibrate(toml::table const& cfg_table, ImageInput const& image_input, std::optional<ImuInput> const& imu_input,
               SqlitePtr const db);

}  // namespace reprojection::application
