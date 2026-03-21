#include "config/validate_target_config.hpp"

#include <toml++/toml.hpp>

#include "toml_helpers.hpp"

namespace reprojection::config {

std::optional<ParserErrorMsg> ValidateTargetConfig(toml::table const& target_cfg) {
    std::map<std::string, TomlType> const required_keys{{"pattern_size", TomlType::Array}, {"type", TomlType::String}};
    if (auto const error_msg{ValidateRequiredKeys(target_cfg, required_keys)}) {
        return error_msg;
    }

    // TODO(Jack): NO IDEA why I need to supress code coverage here. Is something wrong with the maps or is my code here
    //  really not executed in testing?
    std::map<std::string, TomlType> const optional_keys{
        {"circle_grid", TomlType::Table},               // LCOV_EXCL_LINE
        {"circle_grid.asymmetric", TomlType::Boolean},  // LCOV_EXCL_LINE
        {"unit_dimension", TomlType::FloatingPoint}};

    std::map<std::string, TomlType> possible_keys{required_keys};
    possible_keys.insert(optional_keys.begin(), optional_keys.end());

    if (auto const error_msg{ValidatePossibleKeys(target_cfg, possible_keys)}) {
        return error_msg;
    }

    return std::nullopt;
}

}  // namespace reprojection::config
