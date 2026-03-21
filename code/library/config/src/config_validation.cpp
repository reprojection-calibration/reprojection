#include "config/config_validation.hpp"

#include <toml++/toml.hpp>

#include "toml_helpers.hpp"

namespace reprojection::config {

std::optional<ParserErrorMsg> ValidateCalibrationConfig(toml::table const& calibration_cfg) {
    std::map<std::string, TomlType> const required_keys{{"data", TomlType::Table}, {"sensor", TomlType::Table},{"target", TomlType::Table}};

    return ValidateConfigKeys(calibration_cfg, required_keys, {});
}

std::optional<ParserErrorMsg> ValidateTargetConfig(toml::table const& target_cfg) {
    std::map<std::string, TomlType> const required_keys{{"pattern_size", TomlType::Array}, {"type", TomlType::String}};
    std::map<std::string, TomlType> const optional_keys{{"circle_grid", TomlType::Table},
                                                        {"circle_grid.asymmetric", TomlType::Boolean},
                                                        {"unit_dimension", TomlType::FloatingPoint}};

    return ValidateConfigKeys(target_cfg, required_keys, optional_keys);
}



}  // namespace reprojection::config
