#include "config/target_options.hpp"

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
    )"sv,
};

TEST(ConfigTargetOptions, TestParseTargetOptionsGoodConfigs) {
    for (auto const& config : good_configs) {
        toml::table const toml{toml::parse(config)};

        // TODO(Jack): Remove the top level table so we can just pass it in directly! Not needed complexity.
        auto const error_msg{config::ValidateTargetConfig(*toml["target"].as_table())};
        EXPECT_FALSE(error_msg.has_value());
    }
}

std::vector<std::string_view> const bad_configs{
    // An extra random key is present.
    R"(
        [target]
        pattern_size = [3,4]
        type = "circle_grid"
        random_key = "this key should not be here!"
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
        unit_dimension = "this is not a float"
    )"sv,
};

TEST(ConfigTargetOptions, TestParseTargetOptionsBadConfigs) {
    for (auto const& config : bad_configs) {
        toml::table const toml{toml::parse(config)};

        auto const error_msg{config::ValidateTargetConfig(*toml["target"].as_table())};
        EXPECT_TRUE(error_msg.has_value());

        std::cout << error_msg.value().msg << std::endl;
    }
}