#include "application/io.hpp"

#include <gtest/gtest.h>

#include <ranges>

#include "testing_utilities/temporary_file.hpp"

using namespace reprojection;
using TemporaryFile = testing_utilities::TemporaryFile;

TEST(ApplicationIO, TestGetCommandOption) {
    auto result{application::GetCommandOption(nullptr, nullptr, "")};
    EXPECT_FALSE(result.has_value());

    char const arg0[]{"program"};
    char const arg1[]{"--key1"};
    char const arg2[]{"value1"};
    char const arg3[]{"--key2"};
    char const arg4[]{"value2"};

    char const* const argv[]{arg0, arg1, arg2, arg3, arg4};
    int const argc{5};

    result = application::GetCommandOption(argv, argv + argc, "--key1");
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(*result, "value1");

    result = application::GetCommandOption(argv, argv + argc, "--key2");
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(*result, "value2");

    result = application::GetCommandOption(argv, argv + argc, "--nonexistent_key");
    EXPECT_FALSE(result.has_value());
}

TEST(ApplicationIO, TestHappyPath) {
    // TODO(Jack): This is now copy and pasted in three places, should we make one common def in the testing utils?
    static constexpr std::string_view minimum_config{R"(
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

TEST(ApplicationIO, TestBadLoad) {
    auto const result{application::LoadAndValidateConfig("nonexistent.toml")};
    ASSERT_TRUE(std::holds_alternative<TomlErrorMsg>(result));
    EXPECT_EQ(std::get<TomlErrorMsg>(result).error, TomlError::FailedLoad);
    EXPECT_EQ(std::get<TomlErrorMsg>(result).msg,
              "Error parsing file 'nonexistent.toml' - File could not be opened for reading on line (0)");
}

TEST(ApplicationIO, TestBadValidate) {
    static constexpr std::string_view empty_config{R"(
    )"};
    TemporaryFile const config_file{".toml", empty_config};

    auto const result{application::LoadAndValidateConfig(config_file.Path())};
    ASSERT_TRUE(std::holds_alternative<TomlErrorMsg>(result));
    EXPECT_EQ(std::get<TomlErrorMsg>(result).error, TomlError::MissingKey);
    EXPECT_EQ(std::get<TomlErrorMsg>(result).msg, "Configuration does not contain required key: sensor of type: table");
}