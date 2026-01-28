#include "toml_helpers.hpp"

#include <gtest/gtest.h>

using namespace reprojection;
using namespace std::string_view_literals;

TEST(ConfigTomlHelpers, TestValidateTomlHappyPath) {
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
    std::map<std::string, DataType> const required_keys{{"target", DataType::Table}, {"solver", DataType::Table}};
    EXPECT_FALSE(config::ValidateRequiredKeys(config, required_keys).has_value());

    // Example use case #2 - check that a config has specific keys.
    std::map<std::string, DataType> const required_keys_1{{"target.pattern_size", DataType::Array},
                                                          {"target.unit_dimension", DataType::FloatingPoint},
                                                          {"solver.num_threads", DataType::Integer},
                                                          {"solver.minimizer_type", DataType::String}};
    EXPECT_FALSE(config::ValidateRequiredKeys(config, required_keys_1).has_value());
}

TEST(ConfigTomlHelpers, TestValidateTomlIncorrectType) {
    static constexpr std::string_view config_file{R"(
        # A data time is currently not supported so it will trigger an error for all requested type checking.
        config_key = 1979-05-27T00:32:00.999999-07:00
    )"sv};
    toml::table const config{toml::parse(config_file)};

    std::map<std::string, DataType> required_keys{{"config_key", DataType::Array}};
    auto const error_msg{config::ValidateRequiredKeys(config, required_keys)};
    ASSERT_TRUE(error_msg.has_value());
    EXPECT_EQ(error_msg->error, ParseErrorType::IncorrectType);
    EXPECT_EQ(error_msg->msg, "Configuration key: config_key is not of expected type: array");

    required_keys["config_key"] = DataType::FloatingPoint;
    EXPECT_TRUE(config::ValidateRequiredKeys(config, required_keys).has_value());

    required_keys["config_key"] = DataType::Integer;
    EXPECT_TRUE(config::ValidateRequiredKeys(config, required_keys).has_value());

    required_keys["config_key"] = DataType::String;
    EXPECT_TRUE(config::ValidateRequiredKeys(config, required_keys).has_value());

    required_keys["config_key"] = DataType::Table;
    EXPECT_TRUE(config::ValidateRequiredKeys(config, required_keys).has_value());
}

TEST(ConfigTomlHelpers, TestValidateTomlUnknownKey) {
    static constexpr std::string_view config_file{R"(
        config_key = [3,4]
    )"sv};
    toml::table const config{toml::parse(config_file)};

    std::map<std::string, DataType> const required_keys{{"config_key", DataType::Array},
                                                        {"other_key", DataType::Integer}};
    auto const error_msg{config::ValidateRequiredKeys(config, required_keys)};
    ASSERT_TRUE(error_msg.has_value());
    EXPECT_EQ(error_msg->error, ParseErrorType::UnknownKey);
    EXPECT_EQ(error_msg->msg, "Configuration does not contain required key: other_key of type: integer");
}