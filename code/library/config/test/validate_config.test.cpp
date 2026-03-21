#include "config/validate_config.hpp"

#include <gtest/gtest.h>

#include "testing_utilities/temporary_file.hpp"

namespace reprojection::config {

std::optional<ParserErrorMsg> ValidateCalibrationConfig(toml::table const& cfg) {

    return std::nullopt;
}

}  // namespace reprojection::config

using namespace reprojection;
using TemporaryFile = testing_utilities::TemporaryFile;

static constexpr std::string_view minimum_config{R"(
        [data]
        file = "/data/TUM-Visual-Inertial-Dataset/dataset-calib-imu4.bag"

        [sensor]
        camera_name = "/cam0/image_raw"
        camera_model = "double_sphere"

        [target]
        pattern_size = [3,4]
        type = "circle_grid"
    )"};

TEST(XXX, YYY) {
    toml::table const toml{toml::parse(minimum_config)};

    config::ValidateCalibrationConfig(toml);
}