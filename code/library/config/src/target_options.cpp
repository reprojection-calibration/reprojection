#include "target_options.hpp"

#include <toml++/toml.hpp>

#include "toml_helpers.hpp"

namespace reprojection::config {

std::optional<ParserErrorMsg> ValidateTargetConfig(toml::table const& target_cfg) {
    // TODO(Jack): Why does my use of .merge() below not allow this to be const? Makes no sense.
    // TODO(Jack): If this could be const then it should also be static. Also below for the optional_keys.
    std::map<std::string, TomlType> required_keys{{"pattern_size", TomlType::Array}, {"type", TomlType::String}};

    if (auto const error_msg{ValidateRequiredKeys(target_cfg, required_keys)}) {
        return error_msg;
    }

    // TODO(Jack): I wish there was a better way to merge the maps here and express the semantics of what is required,
    //  what is optional, and how the combination of the two is what is possible.
    std::map<std::string, TomlType> optional_keys{{"circle_grid", TomlType::Table},
                                                  {"circle_grid.asymmetric", TomlType::Boolean},
                                                  {"unit_dimension", TomlType::FloatingPoint}};
    optional_keys.merge(required_keys);
    std::map<std::string, TomlType> const possible_keys{optional_keys};

    if (auto const error_msg{ValidatePossibleKeys(target_cfg, possible_keys)}) {
        return error_msg;
    }

    return std::nullopt;
}

}  // namespace reprojection::config
