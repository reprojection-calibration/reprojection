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

struct ImageInput {
    ImageSampler source;
    std::string signature;
};

struct ImuInput {
    ImuSampler source;
    std::string signature;
};

std::optional<AppArgs> ParseArgs(int const argc, char const* const argv[]);

// TODO(Jack): How should we pass the ImageSourceSignature?
void Calibrate(toml::table const& cfg_table, ImageInput const& image_input, std::optional<ImuInput> const& imu_input,
               SqlitePtr const db);

}  // namespace reprojection::application
