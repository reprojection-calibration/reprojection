#include "config/config_validation.hpp"

#include <gtest/gtest.h>

// cppcheck-suppress missingInclude
#include "testing_utilities/generated/minimum_config.hpp"

using namespace reprojection;

TEST(ConfigValidateConfig, TestValidateCalibrationMinimumConfig) {
    toml::table const toml{toml::parse(testing_utilities::minimum_config)};

    auto const result{config::ValidateCalibrationConfig(toml)};
    EXPECT_FALSE(result.has_value());
}

static constexpr std::string_view maximum_config{R"(
        [application]
        show_extraction = true

        [camera]
        sensor_name = "/cam0/image_raw"
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

static constexpr std::string_view invalid_config{R"(
        bad_config = 123
    )"};

TEST(ConfigValidateConfig, TestValidateCalibrationConfiginvalid_config) {
    toml::table const toml{toml::parse(invalid_config)};

    auto const result{config::ValidateCalibrationConfig(toml)};
    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result->error, TomlError::MissingKey);
}