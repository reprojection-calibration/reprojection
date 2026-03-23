#include "application/cli_utils.hpp"

#include <gtest/gtest.h>

#include <ranges>

using namespace reprojection;

TEST(ApplicationCliUtils, TestGetCommandOption) {
    char const arg0[]{"program"};
    char const arg1[]{"--key1"};
    char const arg2[]{"value1"};
    char const arg3[]{"--key2"};
    char const arg4[]{"value2"};

    char const* const argv[]{arg0, arg1, arg2, arg3, arg4};
    int const argc{5};

    auto result{application::GetCommandOption(argv, argv + argc, "--key1")};
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(*result, "value1");

    result = application::GetCommandOption(argv, argv + argc, "--key2");
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(*result, "value2");

    result = application::GetCommandOption(argv, argv + argc, "--nonexistent_key");
    EXPECT_FALSE(result.has_value());
}