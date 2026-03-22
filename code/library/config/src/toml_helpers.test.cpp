#include "toml_helpers.hpp"

#include <gtest/gtest.h>

using namespace reprojection;
using namespace std::string_view_literals;

using TomlType = config::TomlType;
using TomlParseError = config::TomlParseError;

TEST(ConfigTomlHelpers, TestValidateConfigKeys) {
    static constexpr std::string_view config_file{R"(
        [table]
        key1 = "value1"
    )"sv};
    toml::table const config{toml::parse(config_file)};

    // By default, we are strict and do not allow any unknown keys!
    config::TomlKeys const required_keys{{"table", TomlType::Table}};
    auto result{ValidateConfigKeys(config, required_keys)};
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->error, config::TomlParseError::UnknownKey);

    // If we explicitly allow unknown keys then it's no problem.
    result = ValidateConfigKeys(config, required_keys, {}, true);
    EXPECT_FALSE(result.has_value());

    // If we explicitly specify the key as an optional key it is also no problem :)
    config::TomlKeys const optional_keys{{"table.key1", TomlType::String}};
    result = ValidateConfigKeys(config, required_keys, optional_keys);
    EXPECT_FALSE(result.has_value());
}

TEST(ConfigTomlHelpers, TestValidateRequiredKeys) {
    static constexpr std::string_view config_file{R"(
        [table]
        key1 = "value1"
    )"sv};
    toml::table const config{toml::parse(config_file)};

    config::TomlKeys required_keys{{"table", TomlType::Table}, {"table.key1", TomlType::String}};
    auto result{config::ValidateRequiredKeys(config, required_keys)};
    EXPECT_FALSE(result.has_value());

    required_keys = {{"table", TomlType::Table}, {"table.key1", TomlType::String}, {"table.key2", TomlType::String}};
    result = config::ValidateRequiredKeys(config, required_keys);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->error, TomlParseError::MissingKey);

    required_keys = {{"table", TomlType::Table}, {"table.key1", TomlType::Integer}};
    result = config::ValidateRequiredKeys(config, required_keys);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->error, TomlParseError::IncorrectType);
}

TEST(ConfigTomlHelpers, TestGetValidatePossibleKeys) {
    static constexpr std::string_view config_file{R"(
        [table]
        key1 = "value1"
    )"sv};
    toml::table const config{toml::parse(config_file)};

    config::TomlKeys possible_keys{{"table", TomlType::Table}, {"table.key1", TomlType::String}};

    auto result{ValidatePossibleKeys(config, possible_keys, false)};
    EXPECT_FALSE(result.has_value());
    result = ValidatePossibleKeys(config, possible_keys, true);
    EXPECT_FALSE(result.has_value());

    // If we do not allow unknown keys then we will get an UnknownKey error when there are keys in the table not found
    // in the set of possible keys. - this is the core idea.
    possible_keys = {};

    result = ValidatePossibleKeys(config, possible_keys, false);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->error, config::TomlParseError::UnknownKey);
    result = ValidatePossibleKeys(config, possible_keys, true);
    EXPECT_FALSE(result.has_value());

    possible_keys = {{"table", TomlType::Table}, {"table.key1", TomlType::String}, {"table.key2", TomlType::String}};

    result = ValidatePossibleKeys(config, possible_keys, false);
    EXPECT_FALSE(result.has_value());
    result = ValidatePossibleKeys(config, possible_keys, true);
    EXPECT_FALSE(result.has_value());

    possible_keys = {{"table", TomlType::Table}, {"table.key1", TomlType::Integer}};

    result = ValidatePossibleKeys(config, possible_keys, false);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->error, config::TomlParseError::IncorrectType);
    result = ValidatePossibleKeys(config, possible_keys, true);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->error, config::TomlParseError::IncorrectType);
}

TEST(ConfigTomlHelpers, TestGetTomlPaths) {
    static constexpr std::string_view config_file{R"(
        [table]
        key1 = "value1"

        [table.nested]
        key2 = "value2"
    )"sv};
    toml::table const config{toml::parse(config_file)};

    std::vector<std::string> full_path_keys;
    config::GetTomlPaths(config, full_path_keys);
    std::vector<std::string> const gt_full_path_keys{"table.key1", "table.nested.key2", "table.nested", "table"};
    EXPECT_TRUE(full_path_keys == gt_full_path_keys);
}