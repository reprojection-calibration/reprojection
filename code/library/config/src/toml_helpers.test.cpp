#include "toml_helpers.hpp"

#include <gtest/gtest.h>

namespace reprojection::config {

// TEST!!!!
// TEST!!!!
// TEST!!!!
// TEST!!!!
// TODO(Jack): Do this without recursion instead of editing toml_paths in place!
// NOTE(Jack): We need to iterate over the entire table to construct the full toml paths. Unfortunately there is no
// reverse of the .at_path() function which we can just use.
void GetTomlPaths(toml::table const& table, std::vector<std::string>& toml_paths, std::string_view prefix = "") {
    for (auto const& [key, node] : table) {
        std::string const full_path{prefix.empty() ? std::string(key) : std::string(prefix) + "." + std::string(key)};
        if (auto const* sub{node.as_table()}) {
            GetTomlPaths(*sub, toml_paths, full_path);
        }

        toml_paths.push_back(full_path);
    }
}

// TODO(Jack): Would it be useful or more informative to the user to have a version of this that only validates table
//  headers?
std::optional<ParseError> ValidatePossibleKeys(toml::table const& table,
                                               std::map<std::string, DataType> const& possible_keys) {
    std::vector<std::string> full_path_keys;
    GetTomlPaths(table, full_path_keys);

    for (auto const& key : full_path_keys) {
        // TODO CHECK TYPE LIKE WE DO IN THE REQUIRED FUNCTION!!!!
        if (not possible_keys.contains(key)) {
            // TODO FILL OUT THE TYPE IN THE ERROR MESSAGE!!!!
            return ParseError{ParseErrorType::UnknownKey,
                              "Configuration contains an unexpected key: " + key + " of type BLAH"};
        }
    }

    return std::nullopt;
}

}  // namespace reprojection::config

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
    std::map<std::string, DataType> const required_keys{{"target", DataType::Table}, {"solver", DataType::Table}};
    EXPECT_FALSE(config::ValidateRequiredKeys(config, required_keys).has_value());

    // Example use case #2 - check that a config has specific keys.
    std::map<std::string, DataType> const required_keys_1{{"target.pattern_size", DataType::Array},
                                                          {"target.unit_dimension", DataType::FloatingPoint},
                                                          {"solver.num_threads", DataType::Integer},
                                                          {"solver.minimizer_type", DataType::String}};
    EXPECT_FALSE(config::ValidateRequiredKeys(config, required_keys_1).has_value());
}

TEST(ConfigTomlHelpers, TestValidateRequiredKeysIncorrectType) {
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

TEST(ConfigTomlHelpers, TestValidateRequiredKeysMissingKey) {
    static constexpr std::string_view config_file{R"(
        config_key = [3,4]
    )"sv};
    toml::table const config{toml::parse(config_file)};

    std::map<std::string, DataType> const required_keys{{"config_key", DataType::Array},
                                                        {"other_key", DataType::Integer}};
    auto const error_msg{config::ValidateRequiredKeys(config, required_keys)};
    ASSERT_TRUE(error_msg.has_value());
    EXPECT_EQ(error_msg->error, ParseErrorType::MissingKey);
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
    std::map<std::string, DataType> const possible_keys{{"target", DataType::Table},
                                                        {"target.circle_grid", DataType::Table},
                                                        {"target.circle_grid.asymmetric", DataType::Boolean}};
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

    std::map<std::string, DataType> const possible_keys{{"target", DataType::Table},
                                                        {"target.circle_grid", DataType::Table},
                                                        {"target.circle_grid.asymmetric", DataType::Boolean}};
    auto const error_msg{config::ValidatePossibleKeys(config, possible_keys)};
    ASSERT_TRUE(error_msg.has_value());
    EXPECT_EQ(error_msg->error, ParseErrorType::UnknownKey);
    EXPECT_EQ(error_msg->msg,
              "Configuration contains an unexpected key: target.circle_grid.some_random_key of type BLAH");
}