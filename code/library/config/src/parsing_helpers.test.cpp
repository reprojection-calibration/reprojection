#include "parsing_helpers.hpp"

#include <gtest/gtest.h>

using namespace reprojection;

TEST(ConfigParsingHelpers, TestOptionalTable) {
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

TEST(ConfigParsingHelpers, TestRequiredTable) {
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

TEST(ConfigParsingHelpers, TestIndexedKeyIndex) {
    std::string const base{"cam"};
    std::string key{"cam"};

    // A key without an index is not allowed.
    EXPECT_FALSE(config::IndexedKeyIndex(key, base));

    // Bad suffix! Not all digits.
    key = "cam0_dev";
    EXPECT_FALSE(config::IndexedKeyIndex(key, base));

    // Mismatched base.
    key = "CAM";
    EXPECT_FALSE(config::IndexedKeyIndex(key, base));

    key = "cam0";
    std::optional result{config::IndexedKeyIndex(key, base)};
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(*result, 0);

    key = "cam55";
    result = config::IndexedKeyIndex(key, base);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(*result, 55);
}

TEST(ConfigParsingHelpers, TestRequireIndexedTables) {
    static constexpr std::string_view table_content{R"(
        [example0]

        [example4]
    )"};
    toml::table const table{toml::parse(table_content)};

    // Dummy example struct so we do not need to parse a real Config type.
    struct Example {
        static Example Parse(toml::table const& table, int const index) {
            static_cast<void>(table);

            return Example{index};
        }

        int index_;
    };

    auto const result{config::RequireIndexedTables<Example>(table, "example")};
    EXPECT_EQ(std::size(result), 2);
    EXPECT_EQ(result[0].index_, 0);
    EXPECT_EQ(result[1].index_, 4);

    // Requires at least one parsed indexed table.
    EXPECT_THROW(config::RequireIndexedTables<Example>(table, "xyz"), std::runtime_error);
}

TEST(ConfigParsingHelpers, TestRejectUnexpectedKeys) {
    static constexpr std::string_view table_content{R"(
        key1 = "value1"

        [table0]

        [table1]

    )"};
    toml::table const table{toml::parse(table_content)};

    // Throws because key1 is not in the allowed_keys.
    EXPECT_THROW(config::RejectUnexpectedKeys(table, {}, {"table"}, ""), std::runtime_error);

    // Does throw because table (and table1) is not specified explicitly as an indexable key.
    EXPECT_THROW(config::RejectUnexpectedKeys(table, {"key1", "table"}, {}, ""), std::runtime_error);

    // Does not throw because 'key1' and the indexable 'table' key are properly specified.
    EXPECT_NO_THROW(config::RejectUnexpectedKeys(table, {"key1"}, {"table"}, ""));
}

TEST(ConfigParsingHelpers, TestOptional) {
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

TEST(ConfigParsingHelpers, TestRequire) {
    static constexpr std::string_view table_content{R"(
        key1 = "value1"
    )"};
    toml::table const table{toml::parse(table_content)};

    // Key does not exist so we throw.
    EXPECT_THROW(config::Require<std::string>(table, ""), std::runtime_error);

    // Ket exists so we do not throw.
    EXPECT_NO_THROW(config::Require<std::string>(table, "key1"));
}

TEST(ConfigParsingHelpers, TestRequireArray) {
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

TEST(ConfigParsingHelpers, TestOverrideIfPresent) {
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
