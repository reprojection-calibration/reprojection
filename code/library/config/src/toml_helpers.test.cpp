#include "toml_helpers.hpp"

#include <gtest/gtest.h>

using namespace reprojection;
using namespace std::string_view_literals;

TEST(ConfigTomlHelpers, TestValidateRequiredKeysHappyPath) {
    static constexpr std::string_view config_file{R"(
        [target]
        pattern_size = [3,4]
        unit_dimension = 0.5

        [solver]
        num_threads = 1
        minimizer_type = "TRUST_REGION"
    )"sv};
    toml::table const config{toml::parse(config_file)};

    // Example use case #1 - check that a config has specific table sections.
    std::map<std::string, TomlType> const required_keys{{"target", TomlType::Table}, {"solver", TomlType::Table}};
    EXPECT_FALSE(config::ValidateRequiredKeys(config, required_keys).has_value());

    // Example use case #2 - check that a config has specific keys.
    std::map<std::string, TomlType> const required_keys_1{{"target.pattern_size", TomlType::Array},
                                                          {"target.unit_dimension", TomlType::FloatingPoint},
                                                          {"solver.num_threads", TomlType::Integer},
                                                          {"solver.minimizer_type", TomlType::String}};
    EXPECT_FALSE(config::ValidateRequiredKeys(config, required_keys_1).has_value());
}

TEST(ConfigTomlHelpers, TestValidateRequiredKeysIncorrectType) {
    static constexpr std::string_view config_file{R"(
        # A data time is currently not supported so it will trigger an error for all requested type checking.
        config_key = 1979-05-27T00:32:00.999999-07:00
    )"sv};
    toml::table const config{toml::parse(config_file)};

    std::map<std::string, TomlType> required_keys{{"config_key", TomlType::Array}};
    auto const error_msg{config::ValidateRequiredKeys(config, required_keys)};
    ASSERT_TRUE(error_msg.has_value());
    EXPECT_EQ(error_msg->error, TomlParseError::IncorrectType);
    EXPECT_EQ(error_msg->msg, "Configuration key: config_key is not of expected type: array");

    required_keys["config_key"] = TomlType::FloatingPoint;
    EXPECT_TRUE(config::ValidateRequiredKeys(config, required_keys).has_value());

    required_keys["config_key"] = TomlType::Integer;
    EXPECT_TRUE(config::ValidateRequiredKeys(config, required_keys).has_value());

    required_keys["config_key"] = TomlType::String;
    EXPECT_TRUE(config::ValidateRequiredKeys(config, required_keys).has_value());

    required_keys["config_key"] = TomlType::Table;
    EXPECT_TRUE(config::ValidateRequiredKeys(config, required_keys).has_value());
}

TEST(ConfigTomlHelpers, TestValidateRequiredKeysMissingKey) {
    static constexpr std::string_view config_file{R"(
        config_key = [3,4]
    )"sv};
    toml::table const config{toml::parse(config_file)};

    std::map<std::string, TomlType> const required_keys{{"config_key", TomlType::Array},
                                                        {"other_key", TomlType::Integer}};
    auto const error_msg{config::ValidateRequiredKeys(config, required_keys)};
    ASSERT_TRUE(error_msg.has_value());
    EXPECT_EQ(error_msg->error, TomlParseError::MissingKey);
    EXPECT_EQ(error_msg->msg, "Configuration does not contain required key: other_key of type: integer");
}

TEST(ConfigTomlHelpers, TestValidatePossibleKeysHappyPaths) {
    static constexpr std::string_view config_file{R"(
        [target.circle_grid]
        asymmetric = true
    )"sv};
    toml::table const config{toml::parse(config_file)};

    // TODO(Jack): An unfortunate quality of our parsing right now is that having the section header
    //  [target.circle_grid] will actually introduce two possible key paths, first the "target" key and then the
    //  "target.circle_grid" key. If possible we should consider a design which considers an entry like
    //  [target.circle_grid] as one single key. For now we simply add the "target" key here to the possible keys.
    std::map<std::string, TomlType> const possible_keys{{"target", TomlType::Table},
                                                        {"target.circle_grid", TomlType::Table},
                                                        {"target.circle_grid.asymmetric", TomlType::Boolean}};
    EXPECT_FALSE(config::ValidatePossibleKeys(config, possible_keys).has_value());

    // The absence of a possible key is not an error, here we test this with a completely empty config file.
    static constexpr std::string_view empty_config_file{R"()"sv};
    toml::table const empty_config{toml::parse(empty_config_file)};

    EXPECT_FALSE(config::ValidatePossibleKeys(empty_config, possible_keys).has_value());
}

TEST(ConfigTomlHelpers, TestValidatePossibleKeysUnknownKey) {
    static constexpr std::string_view config_file{R"(
        [target.circle_grid]
        asymmetric = true
        some_random_key = 666
    )"sv};
    toml::table const config{toml::parse(config_file)};

    std::map<std::string, TomlType> const possible_keys{{"target", TomlType::Table},
                                                        {"target.circle_grid", TomlType::Table},
                                                        {"target.circle_grid.asymmetric", TomlType::Boolean}};
    auto const error_msg{config::ValidatePossibleKeys(config, possible_keys)};
    ASSERT_TRUE(error_msg.has_value());
    EXPECT_EQ(error_msg->error, TomlParseError::UnknownKey);
    EXPECT_EQ(error_msg->msg, "Configuration contains an unexpected key: target.circle_grid.some_random_key");
}

TEST(ConfigTomlHelpers, TestValidatePossibleKeysIncorrectType) {
    static constexpr std::string_view config_file{R"(
        [target.circle_grid]
        asymmetric = 123
    )"sv};
    toml::table const config{toml::parse(config_file)};

    std::map<std::string, TomlType> const possible_keys{{"target", TomlType::Table},
                                                        {"target.circle_grid", TomlType::Table},
                                                        {"target.circle_grid.asymmetric", TomlType::Boolean}};
    auto const error_msg{config::ValidatePossibleKeys(config, possible_keys)};
    ASSERT_TRUE(error_msg.has_value());
    EXPECT_EQ(error_msg->error, TomlParseError::IncorrectType);
    EXPECT_EQ(error_msg->msg, "Configuration key: target.circle_grid.asymmetric is not of expected type: boolean");
}