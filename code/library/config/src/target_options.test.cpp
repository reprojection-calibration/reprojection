#include <gtest/gtest.h>

#include <string_view>
#include <toml++/toml.hpp>

#include "target_enum_parsing.hpp"

namespace reprojection::config {

bool ValidateTargetConfig(toml::table target_cfg) {
    if (auto const node{target_cfg.get("pattern_size")}) {
        if (node->as_array()->size() != 2) {
            return false;
        }
    } else {
        return false;
    }
    target_cfg.erase("pattern_size");

    if (not target_cfg.get("type")->value<std::string>()) {
        return false;
    }
    std::string const target_type{target_cfg["type"].as_string()->get()};
    target_cfg.erase("type");

    // NOTE(Jack): Not having a unit_dimension is acceptable. If it is not provided we should/will just assume a value
    // of one. But if it is present it must be of float type.
    if (auto const node{target_cfg.get("unit_dimension")}) {
        if (not node->as_floating_point()) {
            return false;
        }
        target_cfg.erase("unit_dimension");
    }

    // NOTE(Jack): At time of writing circle grid is the only target type which has additional options.
    if (StringToTargetTypeEnum(target_type) == TargetType::CircleGrid) {
        if (auto const node{target_cfg.at_path("circle_grid.asymmetric")}) {
            if (not node.as_boolean()) {
                return false;
            }
            // THIS IS SUPER MESSY LOGIC - I NEED TO PRACTICE MORE WITH TOMLPLUSPLUS
            target_cfg.get("circle_grid")->as_table()->erase("asymmetric");

            if (not target_cfg["circle_grid"].as_table()->empty()) {
                return false;
            }
            target_cfg.erase("circle_grid");
        }
    }

    if (not target_cfg.empty()) {
        std::ostringstream oss;
        oss << "Unexpected parameters found in the configuration file, are you sure they are correct?\n";
        for (const auto& [key, _] : target_cfg) {
            oss << "  - " << key.str() << "\n";
        }

        throw std::runtime_error(oss.str());
    }

    return true;
}

}  // namespace reprojection::config

using namespace reprojection;
using namespace std::string_view_literals;

std::vector<std::string_view> const good_configs{
    // Minimum viable config - in this case unit_dimension should default to 1
    R"(
        [target]
        pattern_size = [3,4]
        type = "circle_grid"
    )"sv,
    // The three allowed keys for the top level target config
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

TEST(ConfigTargetOptions, TestParseTargetOptionsBadConfigs) {
    // pattern_size is not length two - it should only contain the number of rows and columns, nothing more, nothing
    // less.
    static constexpr std::string_view config_file{R"(
        [target]
        pattern_size = [3,4,5]
        type = "april_grid3"
    )"sv};
    toml::table config{toml::parse(config_file)};
    bool valid_target_config{config::ValidateTargetConfig(*config["target"].as_table())};
    EXPECT_FALSE(valid_target_config);

    // type is not string.
    static constexpr std::string_view config_file_1{R"(
        [target]
        pattern_size = [3,4]
        type = 123
    )"sv};
    config = toml::parse(config_file_1);
    valid_target_config = config::ValidateTargetConfig(*config["target"].as_table());
    EXPECT_FALSE(valid_target_config);

    // unit_dimension is not float type.
    static constexpr std::string_view config_file_2{R"(
        [target]
        pattern_size = [3,4]
        type = "april_grid3"
        unit_dimension = "bad_type"
    )"sv};
    config = toml::parse(config_file_2);
    valid_target_config = config::ValidateTargetConfig(*config["target"].as_table());
    EXPECT_FALSE(valid_target_config);
}

// Because the selected target type is april_grid3, the [target.circle_grid] section should not be present.
TEST(ConfigTargetOptions, TestParseTargetOptionsCircleGridMismatch) {
    static constexpr std::string_view config_file{R"(
        [target]
        pattern_size = [3,4]
        type = "april_grid3"
        unit_dimension = 0.5

        [target.circle_grid]
        asymmetric = true
    )"sv};
    toml::table const config{toml::parse(config_file)};

    EXPECT_THROW(config::ValidateTargetConfig(*config["target"].as_table()), std::runtime_error);
}