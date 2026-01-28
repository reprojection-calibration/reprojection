#include "target_options.hpp"

#include <gtest/gtest.h>

#include <string_view>
#include <toml++/toml.hpp>

using namespace reprojection;
using namespace std::string_view_literals;

std::vector<std::string_view> const good_configs{
    // Minimum viable config - in this case unit_dimension should default to 1.
    R"(
        [target]
        pattern_size = [3,4]
        type = "circle_grid"
    )"sv,
    // The three allowed keys for the top level target config.
    R"(
        [target]
        pattern_size = [3,4]
        type = "circle_grid"
        unit_dimension = 0.5
    )"sv,
    // For circle grid targets only, an additional [target.circle_grid] section is allowed to specify the asymmetric
    // property. At time of writing "asymmetric" is the only supported key, and it is only valid for circle grid
    // targets.
    R"(
        [target]
        pattern_size = [3,4]
        type = "circle_grid"
        unit_dimension = 0.5

        [target.circle_grid]
        asymmetric = true
    )"sv};

TEST(ConfigTargetOptions, TestParseTargetOptionsGoodConfigs) {
    for (auto const& config : good_configs) {
        toml::table const toml{toml::parse(config)};

        bool const valid_target_config{config::ValidateTargetConfig(*toml["target"].as_table())};
        EXPECT_TRUE(valid_target_config);
    }
}

// These will return false
std::vector<std::string_view> const bad_configs{
    // An extra random key is present.
    R"(
        [target]
        pattern_size = [3,4]
        type = "circle_grid"
        random_key = 123
    )"sv,
    // The pattern_size field must be an array.
    R"(
        [target]
        pattern_size = "this is not an array"
        type = "circle_grid"
    )"sv,
    // The type field must be a string.
    R"(
        [target]
        pattern_size = [3,4]
        type = 123
    )"sv,
    // The unit_dimension field must be a float/double.
    R"(
        [target]
        pattern_size = [3,4]
        type = "circle_grid"
        unit_dimension = "bad_type"
    )"sv};

TEST(ConfigTargetOptions, TestParseTargetOptionsBadConfigs) {
    for (auto const& config : bad_configs) {
        toml::table const toml{toml::parse(config)};

        bool const valid_target_config{config::ValidateTargetConfig(*toml["target"].as_table())};
        EXPECT_FALSE(valid_target_config);
    }
}