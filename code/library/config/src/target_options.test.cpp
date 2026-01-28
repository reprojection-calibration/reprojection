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
    // Pattern size must have size==2.
    R"(
        [target]
        pattern_size = [3]
        type = "circle_grid"
    )"sv,
    // TODO(Jack): Also check that it is one of the valid types without throwing?
    // The type field must be a string.
    R"(
        [target]
        pattern_size = [3,4]
        type = 123
    )"sv,
    // The unit_dimension field must be a float/double - cannot be a string type.
    R"(
        [target]
        pattern_size = [3,4]
        type = "april_grid3"
        unit_dimension = "bad_type"
    )"sv};

TEST(ConfigTargetOptions, TestParseTargetOptionsBadConfigs) {
    for (auto const& config : bad_configs) {
        toml::table const toml{toml::parse(config)};

        bool const valid_target_config{config::ValidateTargetConfig(*toml["target"].as_table())};
        EXPECT_FALSE(valid_target_config);
    }
}

// These configs will result in an error that throws, this might be a good or bad idea, not sure - but probably bad.
std::vector<std::string_view> const invalid_configs{
    // Unexpected keys are a failing error! This is a strict policy that might reduce usability and we might regret.
    R"(
        [target]
        pattern_size = [3,4]
        type = "circle_grid"
        random_key = "why is this here?"
    )"sv,
    // The [target.circle_grid] is only allowed to be present when type = "circle_grid".
    R"(
        [target]
        pattern_size = [3,4]
        type = "april_grid3"
        unit_dimension = 0.5

        [target.circle_grid]
        asymmetric = true
    )"sv,
    // Also invalid when an unexpected key is in the [target.circle_grid] section.
    R"(
        [target]
        pattern_size = [3,4]
        type = "circle_grid"
        unit_dimension = 0.5

        [target.circle_grid]
        asymmetric = true
        random_key = "why is this here?"
    )"sv};

// Because the selected target type is april_grid3, the [target.circle_grid] section should not be present.
TEST(ConfigTargetOptions, TestParseTargetOptionsInvalidConfigs) {
    for (auto const& config : invalid_configs) {
        toml::table const toml{toml::parse(config)};

        EXPECT_THROW(config::ValidateTargetConfig(*toml["target"].as_table()), std::runtime_error);
    }
}