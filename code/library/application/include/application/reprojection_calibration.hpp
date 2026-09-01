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

struct ImageInput {
    ImageSampler source;
    std::string signature;
};

// A map of image inputs indexed but their sensor_name.
// TODO(Jack): Wish there was a structural way to ensure that the map keys here were the sensor names as taken from the
// parsed calibration config.
using ImageInputs = std::map<std::string, ImageInput>;

// TODO(Jack): Once we get the app running in unit testing with test imu data we can remove this coverage exclusion!
struct ImuInput {  // LCOV_EXCL_LINE
    ImuSampler source;
    std::string signature;
};

std::optional<AppArgs> ParseArgs(int const argc, char const* const argv[]);

Sensors ParseSensors(toml::table const& cfg_table);

void Calibrate(toml::table const& cfg_table, ImageInputs const& image_inputs, std::optional<ImuInput> const& imu_input,
               SqlitePtr db);

}  // namespace reprojection::application
