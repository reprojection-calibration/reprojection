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
    // of one.
    if (auto const node{target_cfg.get("unit_dimension")}) {
        if (not node->as_floating_point()) {
            return false;
        }
    }

    // NOTE(Jack): At time of writing circle grid is the only target type which has additional options.
    if (StringToTargetTypeEnum(target_type) == TargetType::CircleGrid) {
        if (auto const value{target_cfg.get("circle_grid.asymmetric")}) {
            if (not value->as_boolean()) {
                return false;
            }
            target_cfg.erase("circle_grid.asymmetric");
        }

        if (not target_cfg["circle_grid"].as_table()->empty()) {
            return false;
        }
        target_cfg.erase("circle_grid");
    }

    return true;
}

}  // namespace reprojection::config

using namespace reprojection;
using namespace std::string_view_literals;

TEST(ConfigTargetOptions, TestParseTargetOptionsHappyPath) {
    static constexpr std::string_view config_file{R"(
        [target]
        pattern_size = [3,4]
        type = "april_grid3"
        unit_dimension = 0.5

        [target.circle_grid]
        asymmetric = true
    )"sv};
    toml::table const config{toml::parse(config_file)};

    bool const valid_target_config{config::ValidateTargetConfig(*config["target"].as_table())};
    ASSERT_TRUE(valid_target_config);
}
