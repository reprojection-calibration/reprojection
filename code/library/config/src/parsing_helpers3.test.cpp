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

TEST(ConfigParsingHelpers3, TestRejectUnexpectedKeys) {
    static constexpr std::string_view table_content{R"(
        key1 = "value1"

        [table1]
        key2 = "value2"
    )"};
    toml::table const table{toml::parse(table_content)};

    // Throws because key1 is not in the allowed_keys
    EXPECT_THROW(config::RejectUnexpectedKeys(table, {"table1"}, ""), std::runtime_error);

    // Does not throw because both keys are in allowed_keys. Note that this method does not recurse into the child
    // tables, therefore we do not need to specify key2 as part of the allowed_keys.
    EXPECT_NO_THROW(config::RejectUnexpectedKeys(table, {"key1", "table1"}, ""));
}