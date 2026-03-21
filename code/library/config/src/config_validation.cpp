#include "config/config_validation.hpp"

#include <toml++/toml.hpp>

#include "toml_helpers.hpp"

namespace reprojection::config {

std::optional<ParserErrorMsg> ValidateCalibrationConfig(toml::table const& calibration_cfg) {
    std::map<std::string, TomlType> const required_tables{
        {"data", TomlType::Table}, {"sensor", TomlType::Table}, {"target", TomlType::Table}};
    if (auto const error_msg{ValidateConfigKeys(calibration_cfg, required_tables, {}, true)}) {
        return error_msg;  // LCOV_EXCL_LINE
    }

    if (auto const error_msg{ValidateDataConfig(*calibration_cfg["data"].as_table())}) {
        return error_msg;  // LCOV_EXCL_LINE
    }
    if (auto const error_msg{ValidateSensorConfig(*calibration_cfg["sensor"].as_table())}) {
        return error_msg;  // LCOV_EXCL_LINE
    }
    if (auto const error_msg{ValidateTargetConfig(*calibration_cfg["target"].as_table())}) {
        return error_msg;  // LCOV_EXCL_LINE
    }

    return std::nullopt;
}

std::optional<ParserErrorMsg> ValidateDataConfig(toml::table const& data_cfg) {
    std::map<std::string, TomlType> const required_keys{{"file", TomlType::String}};

    return ValidateConfigKeys(data_cfg, required_keys);
}

std::optional<ParserErrorMsg> ValidateSensorConfig(toml::table const& sensor_cfg) {
    std::map<std::string, TomlType> const required_keys{{"camera_name", TomlType::String},  // LCOV_EXCL_LINE
                                                        {"camera_model", TomlType::String}};

    return ValidateConfigKeys(sensor_cfg, required_keys);
}

std::optional<ParserErrorMsg> ValidateTargetConfig(toml::table const& target_cfg) {
    std::map<std::string, TomlType> const required_keys{{"pattern_size", TomlType::Array}, {"type", TomlType::String}};
    std::map<std::string, TomlType> const optional_keys{
        {"circle_grid", TomlType::Table},               // LCOV_EXCL_LINE
        {"circle_grid.asymmetric", TomlType::Boolean},  // LCOV_EXCL_LINE
        {"unit_dimension", TomlType::FloatingPoint}};

    return ValidateConfigKeys(target_cfg, required_keys, optional_keys);
}

}  // namespace reprojection::config
