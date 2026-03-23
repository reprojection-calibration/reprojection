#include <gtest/gtest.h>

#include "config/config_validation.hpp"
#include "testing_utilities/temporary_file.hpp"

using namespace reprojection;
using TemporaryFile = testing_utilities::TemporaryFile;

static constexpr std::string_view minimum_config{R"(
        [sensor]
        camera_name = "/cam0/image_raw"
        camera_model = "double_sphere"

        [target]
        pattern_size = [3,4]
        type = "circle_grid"
    )"};

TEST(ConfigValidateConfig, TestValidateCalibrationConfigHappyPath) {
    toml::table const toml{toml::parse(minimum_config)};

    auto const result{config::ValidateCalibrationConfig(toml)};
    EXPECT_FALSE(result.has_value());
}

TEST(ConfigValidateConfig, TestValidateSensorConfig) {
    static constexpr std::string_view good_config{R"(
        camera_name = "/cam0/image_raw"
        camera_model = "double_sphere"
    )"};
    toml::table toml{toml::parse(good_config)};
    auto result{config::ValidateSensorConfig(toml)};
    EXPECT_FALSE(result.has_value());

    static constexpr std::string_view missing_key{R"(
    )"};
    toml = toml::parse(missing_key);
    result = config::ValidateSensorConfig(toml);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->error, TomlError::MissingKey);

    static constexpr std::string_view unknown_key{R"(
        camera_name = "/cam0/image_raw"
        camera_model = "double_sphere"
        unknown_key = 0
    )"};
    toml = toml::parse(unknown_key);
    result = config::ValidateSensorConfig(toml);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->error, TomlError::UnknownKey);
}