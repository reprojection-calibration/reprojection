#include "parsing_helpers.hpp"

#include <gtest/gtest.h>

using namespace reprojection;
using namespace std::string_view_literals;

TEST(ConfigParsingHelpers, TestExtractValueHappyPath) {
    static constexpr std::string_view config_file{R"(
        key1 = "value1"
    )"sv};
    toml::table config{toml::parse(config_file)};

    auto const result{config::ExtractValue<std::string>("key1", config)};
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(*result, "value1");
}

TEST(ConfigParsingHelpers, TestExtractValueMismatchedType) {
    static constexpr std::string_view config_file{R"(
        key1 = "value1"
    )"sv};
    toml::table config{toml::parse(config_file)};

    // Ask for a double when the value is really a string.
    auto const result{config::ExtractValue<double>("key1", config)};
    EXPECT_FALSE(result.has_value());
}