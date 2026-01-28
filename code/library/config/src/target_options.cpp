#include "target_options.hpp"

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

            // TODO UNIFY THIS POLICY OF WHAT TO DO WHEN WE GET KEYS WE DONT WANT
            if (not target_cfg["circle_grid"].as_table()->empty()) {
                throw std::runtime_error("");
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
