#include "application/cli_utils.hpp"

#include <gtest/gtest.h>

#include <ranges>

using namespace reprojection;

TEST(ApplicationCliUtils, TestParseCommandLineInput) {
    auto result{application::ParseCommandLineInput(0, nullptr)};
    ASSERT_TRUE(std::holds_alternative<application::CliErrorMsg>(result));
    EXPECT_EQ(std::get<application::CliErrorMsg>(result).msg, "Missing --config flag");

    char const arg0[]{"program"};
    char const arg1[]{"--config"};
    char const arg2[]{"tmp/config.toml"};
    char const arg3[]{"--data"};
    char const arg4[]{"tmp/data.bag"};

    char const* const argv[]{arg0, arg1, arg2, arg3, arg4};
    int const argc{5};

    result = application::ParseCommandLineInput(argc,argv);
    ASSERT_TRUE(std::holds_alternative<application::PathConfig>(result));

    auto const paths{std::get<application::PathConfig>(result)};
    EXPECT_EQ(paths.config_path, "tmp/config.toml");
    EXPECT_EQ(paths.data_path, "tmp/data.bag");
    EXPECT_EQ(paths.workspace_dir, "tmp");
}

TEST(ApplicationCliUtils, TestGetCommandOption) {
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