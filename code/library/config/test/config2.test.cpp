#include "config/config2.hpp"

#include <gtest/gtest.h>

#include "testing_utilities/temporary_file.hpp"

using namespace reprojection;

TEST(ConfigConfig2, TestConfigApplicationParse) {
    static constexpr std::string_view empty_table{R"(
    )"};
    toml::table toml{toml::parse(empty_table)};

    // Parsing an empty table gives us the default values.
    auto result{config::Config::Application::Parse(toml)};
    EXPECT_TRUE(std::holds_alternative<config::Config::Application>(result));
    EXPECT_EQ(std::get<config::Config::Application>(result).show_extraction, false);
    EXPECT_GE(std::get<config::Config::Application>(result).threads, 1);

    static constexpr std::string_view full_table{R"(
        show_extraction = true
        threads = 5
    )"};
    toml = toml::parse(full_table);

    // Parsing a full table lets us specify all values.
    result = config::Config::Application::Parse(toml);
    EXPECT_TRUE(std::holds_alternative<config::Config::Application>(result));
    EXPECT_EQ(std::get<config::Config::Application>(result).show_extraction, true);
    EXPECT_EQ(std::get<config::Config::Application>(result).threads, 5);

    static constexpr std::string_view unwanted_keys_table{R"(
        show_extraction = true
        threads = 5
        key1 = "value1"
        key2 = [1, 2]
    )"};
    toml = toml::parse(unwanted_keys_table);

    result = config::Config::Application::Parse(toml);
    EXPECT_TRUE(std::holds_alternative<TomlErrorMsg>(result));
    EXPECT_EQ(std::get<TomlErrorMsg>(result).type, TomlError::UnknownKey);
    EXPECT_EQ(std::get<TomlErrorMsg>(result).msg, "{'key1': 'value1', 'key2': [ 1, 2 ]}");
}
