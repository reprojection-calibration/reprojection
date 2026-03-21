#include <gtest/gtest.h>

#include "config/config_validation.hpp"
#include "testing_utilities/temporary_file.hpp"

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

TEST(ConfigValidateConfig, TestValidateCalibrationConfigHappyPath) {
    toml::table const toml{toml::parse(minimum_config)};

    auto const result{config::ValidateCalibrationConfig(toml)};
    EXPECT_FALSE(result.has_value());
}

TEST(ConfigValidateConfig, TestValidateDataConfig) {
    static constexpr std::string_view good_config{R"(
        [data]
        file = "/data/TUM-Visual-Inertial-Dataset/dataset-calib-imu4.bag"
    )"};
    toml::table toml{toml::parse(good_config)};
    auto result{config::ValidateDataConfig(*toml["data"].as_table())};
    EXPECT_FALSE(result.has_value());

    static constexpr std::string_view incorrect_type{R"(
        [data]
        file = 123
    )"};
    toml = toml::parse(incorrect_type);
    result = config::ValidateDataConfig(*toml["data"].as_table());
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->error, config::TomlParseError::IncorrectType);

    static constexpr std::string_view missing_key{R"(
        [data]
    )"};
    toml = toml::parse(missing_key);
    result = config::ValidateDataConfig(*toml["data"].as_table());
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->error, config::TomlParseError::MissingKey);

    static constexpr std::string_view unknown_key{R"(
        [data]
        file = "/data/TUM-Visual-Inertial-Dataset/dataset-calib-imu4.bag"
        unknown_key = 0
    )"};
    toml = toml::parse(unknown_key);
    result = config::ValidateDataConfig(*toml["data"].as_table());
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->error, config::TomlParseError::UnknownKey);
}