#include "application/load_and_validate_config.hpp"

#include <gtest/gtest.h>

#include "testing_utilities/temporary_file.hpp"

using namespace reprojection;
using TemporaryFile = testing_utilities::TemporaryFile;

TEST(ApplicationLoadAndValidateConfig, TestHappyPath) {
    // TODO(Jack): This is now copy and pasted in two places, should we make one common def in the testing utils?
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
    TemporaryFile const config_file{".toml", minimum_config};

    auto const result{application::LoadAndValidateConfig(config_file.Path())};
    ASSERT_TRUE(std::holds_alternative<toml::table>(result));
    toml::table const gt_result{toml::parse(minimum_config)};
    EXPECT_EQ(std::get<toml::table>(result), gt_result);
}

TEST(ApplicationLoadAndValidateConfig, TestBadLoad) {
    auto const result{application::LoadAndValidateConfig("nonexistent.toml")};
    ASSERT_TRUE(std::holds_alternative<TomlErrorMsg>(result));
    EXPECT_EQ(std::get<TomlErrorMsg>(result).error, TomlError::FailedLoad);
    EXPECT_EQ(std::get<TomlErrorMsg>(result).msg,
              "Error parsing file 'nonexistent.toml' - File could not be opened for reading on line (0)");
}

TEST(ApplicationLoadAndValidateConfig, TestBadValidate) {
    static constexpr std::string_view empty_config{R"(
    )"};
    TemporaryFile const config_file{".toml", empty_config};

    auto const result{application::LoadAndValidateConfig(config_file.Path())};
    ASSERT_TRUE(std::holds_alternative<TomlErrorMsg>(result));
    EXPECT_EQ(std::get<TomlErrorMsg>(result).error, TomlError::MissingKey);
    EXPECT_EQ(std::get<TomlErrorMsg>(result).msg, "Configuration does not contain required key: data of type: table");
}