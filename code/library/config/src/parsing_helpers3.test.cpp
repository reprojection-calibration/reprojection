#include "parsing_helpers3.hpp"

#include <gtest/gtest.h>

using namespace reprojection;

TEST(ConfigParsingHelpers3, TestOptionalTable) {
    static constexpr std::string_view table_content{R"(
        key1 = "value1"

        [table1]
        key2 = "value2"
    )"};
    toml::table const table{toml::parse(table_content)};

    // Key does not exist - no table
    auto result{config::OptionalTable(table, "")};
    EXPECT_FALSE(result.has_value());

    // Key exists and is a table
    result = config::OptionalTable(table, "table1");
    ASSERT_TRUE(result.has_value());

    // Key exists but is not a table
    EXPECT_THROW(config::OptionalTable(table, "key1"), std::runtime_error);
}

TEST(ConfigParsingHelpers3, TestRequiredTable) {
    static constexpr std::string_view table_content{R"(
        key1 = "value1"

        [table1]
        key2 = "value2"
    )"};
    toml::table const table{toml::parse(table_content)};

    // Table does not exist so we throw
    EXPECT_THROW(config::RequireTable(table, ""), std::runtime_error);

    // Table exists so we do not throw
    EXPECT_NO_THROW(config::RequireTable(table, "table1"));
}