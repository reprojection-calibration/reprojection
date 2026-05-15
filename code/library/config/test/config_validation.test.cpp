#include "config/config_validation.hpp"

#include <gtest/gtest.h>

using namespace reprojection;

static constexpr std::string_view minimum_config{R"(
        [sensor]
        camera_name = "/cam0/image_raw"
        camera_model = "double_sphere"

        [target]
        pattern_size = [3,4]
        type = "circle_grid"
    )"};

TEST(ConfigValidateConfig, TestValidateCalibrationMinimumConfig) {
    toml::table const toml{toml::parse(minimum_config)};

    auto const result{config::ValidateCalibrationConfig(toml)};
    EXPECT_FALSE(result.has_value());
}

static constexpr std::string_view maximum_config{R"(
        [application]
        show_extraction = true

        [sensor]
        camera_name = "/cam0/image_raw"
        camera_model = "double_sphere"

        [target]
        pattern_size = [3,4]
        type = "circle_grid"
        unit_dimension = 0.05

        [target.circle_grid]
        asymmetric = true
    )"};

TEST(ConfigValidateConfig, TestValidateCalibrationConfigMaximumConfig) {
    toml::table const toml{toml::parse(maximum_config)};

    auto const result{config::ValidateCalibrationConfig(toml)};
    EXPECT_FALSE(result.has_value());
}