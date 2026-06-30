#include "parsing_helpers.hpp"

#include <gtest/gtest.h>

using namespace reprojection;

TEST(ConfigParsingHelpers3, TestOptionalTable) {
    static constexpr std::string_view table_content{R"(
        key1 = "value1"

        [table1]
    )"};
    toml::table const table{toml::parse(table_content)};

    // Key does not exist - no table.
    auto result{config::OptionalTable(table, "")};
    EXPECT_FALSE(result.has_value());

    // Key exists and is a table.
    result = config::OptionalTable(table, "table1");
    ASSERT_TRUE(result.has_value());

    // Key exists but is not a table.
    EXPECT_THROW(config::OptionalTable(table, "key1"), std::runtime_error);
}

TEST(ConfigParsingHelpers3, TestRequiredTable) {
    static constexpr std::string_view table_content{R"(
        key1 = "value1"

        [table1]
    )"};
    toml::table const table{toml::parse(table_content)};

    // Table does not exist so we throw.
    EXPECT_THROW(config::RequireTable(table, ""), std::runtime_error);

    // Table exists so we do not throw.
    EXPECT_NO_THROW(config::RequireTable(table, "table1"));
}

TEST(ConfigParsingHelpers3, TestRejectUnexpectedKeys) {
    static constexpr std::string_view table_content{R"(
        key1 = "value1"

        [table1]
        key2 = "value2"
    )"};
    toml::table const table{toml::parse(table_content)};

    // Throws because key1 is not in the allowed_keys.
    EXPECT_THROW(config::RejectUnexpectedKeys(table, {"table1"}, ""), std::runtime_error);

    // Does not throw because both keys are in allowed_keys. Note that this method does not recurse into the child
    // tables, therefore we do not need to specify key2 as part of the allowed_keys.
    EXPECT_NO_THROW(config::RejectUnexpectedKeys(table, {"key1", "table1"}, ""));
}

TEST(ConfigParsingHelpers3, TestOptional) {
    static constexpr std::string_view table_content{R"(
        key1 = "value1"
        key2 = 1.2
    )"};
    toml::table const table{toml::parse(table_content)};

    // Key does not exist.
    auto result{config::Optional<std::string>(table, "")};
    EXPECT_FALSE(result.has_value());

    // Key exists and has the right type.
    result = config::Optional<std::string>(table, "key1");
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(*result, "value1");

    // Key exists but has the wrong type.
    EXPECT_THROW(config::Optional<std::string>(table, "key2"), std::runtime_error);
}

TEST(ConfigParsingHelpers3, TestRequire) {
    static constexpr std::string_view table_content{R"(
        key1 = "value1"
    )"};
    toml::table const table{toml::parse(table_content)};

    // Key does not exist so we throw.
    EXPECT_THROW(config::Require<std::string>(table, ""), std::runtime_error);

    // Ket exists so we do not throw.
    EXPECT_NO_THROW(config::Require<std::string>(table, "key1"));
}

TEST(ConfigParsingHelpers3, TestRequireArray) {
    static constexpr std::string_view table_content{R"(
        key1 = [1, 22]
        key2 = "value2"
    )"};
    toml::table const table{toml::parse(table_content)};

    // Key does not exist so we throw.
    // NOTE(Jack): We need to put it in an extra parenthesis because otherwise the gtest macro complains.
    EXPECT_THROW((config::RequireArray<int, 2>(table, "")), std::runtime_error);

    // Key is not of array type
    EXPECT_THROW((config::RequireArray<int, 2>(table, "key2")), std::runtime_error);

    // Requested array size does not match.
    EXPECT_THROW((config::RequireArray<int, 10>(table, "key1")), std::runtime_error);

    // Requested array type does not match.
    EXPECT_THROW((config::RequireArray<std::string, 2>(table, "key1")), std::runtime_error);

    // Happy path :)
    auto const result{config::RequireArray<int, 2>(table, "key1")};
    EXPECT_EQ(result[0], 1);
    EXPECT_EQ(result[1], 22);
}

TEST(ConfigParsingHelpers3, TestOverrideIfPresent) {
    static constexpr std::string_view table_content{R"(
        key1 = "value1"
    )"};
    toml::table const table{toml::parse(table_content)};

    std::string value{"xyz"};

    // Key is not present so we get no override.
    config::OverrideIfPresent(table, "", value);
    EXPECT_EQ(value, "xyz");

    // Key is present so we get an override.
    config::OverrideIfPresent(table, "key1", value);
    EXPECT_EQ(value, "value1");
}