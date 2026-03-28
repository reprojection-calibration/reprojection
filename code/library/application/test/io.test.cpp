#include "application/io.hpp"

#include <gtest/gtest.h>

#include <ranges>

#include "testing_utilities/temporary_file.hpp"

using namespace reprojection;
using TemporaryFile = testing_utilities::TemporaryFile;

TEST(ApplicationIO, TestParseCommandLineInput) {
    auto result{application::ParseCommandLineInput(0, nullptr)};
    EXPECT_FALSE(result.has_value());

    char const arg0[]{"program"};
    char const arg1[]{"--config"};
    char const arg2[]{"tmp/config.toml"};
    char const arg3[]{"--data"};
    char const arg4[]{"tmp/data.bag"};
    char const arg5[]{"--workspace"};
    char const arg6[]{"tmp/workspace/"};
    char const* const argv[]{arg0, arg1, arg2, arg3, arg4, arg5, arg6};

    int argc{3};
    result = application::ParseCommandLineInput(argc, argv);
    EXPECT_FALSE(result.has_value());

    argc = 5;
    result = application::ParseCommandLineInput(argc, argv);
    ASSERT_TRUE(result.has_value());

    EXPECT_EQ(result->config_path, "tmp/config.toml");
    EXPECT_EQ(result->data_path, "tmp/data.bag");
    EXPECT_EQ(result->workspace_dir, "tmp");

    argc = 7;
    result = application::ParseCommandLineInput(argc, argv);
    ASSERT_TRUE(result.has_value());

    EXPECT_EQ(result->config_path, "tmp/config.toml");
    EXPECT_EQ(result->data_path, "tmp/data.bag");
    EXPECT_EQ(result->workspace_dir, "tmp/workspace/");
}

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
    ASSERT_TRUE(result.has_value());
    toml::table const gt_result{toml::parse(minimum_config)};
    EXPECT_EQ(*result, gt_result);
}

TEST(ApplicationIO, TestBadLoad) {
    auto const result{application::LoadAndValidateConfig("nonexistent.toml")};
    EXPECT_FALSE(result.has_value());
}

TEST(ApplicationIO, TestBadValidate) {
    static constexpr std::string_view empty_config{R"(
    )"};
    TemporaryFile const config_file{".toml", empty_config};

    auto const result{application::LoadAndValidateConfig(config_file.Path())};
    EXPECT_FALSE(result.has_value());
}