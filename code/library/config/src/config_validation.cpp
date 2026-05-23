#include "config/config_validation.hpp"

#include <toml++/toml.hpp>

#include "toml_helpers.hpp"

namespace reprojection::config {

std::optional<TomlErrorMsg> ValidateCalibrationConfig(toml::table const& calibration_cfg) {
    std::map<std::string, TomlType> const required_keys{{"camera", TomlType::Table},                // LCOV_EXCL_LINE
                                                        {"camera.camera_model", TomlType::String},  // LCOV_EXCL_LINE
                                                        {"camera.sensor_name", TomlType::String},   // LCOV_EXCL_LINE
                                                        {"target", TomlType::Table},                // LCOV_EXCL_LINE
                                                        {"target.pattern_size", TomlType::Array},   // LCOV_EXCL_LINE
                                                        {"target.type", TomlType::String}};

    std::map<std::string, TomlType> const optional_keys{
        {"application", TomlType::Table},                      // LCOV_EXCL_LINE
        {"application.show_extraction", TomlType::Boolean},    // LCOV_EXCL_LINE
        {"target.circle_grid", TomlType::Table},               // LCOV_EXCL_LINE
        {"target.circle_grid.asymmetric", TomlType::Boolean},  // LCOV_EXCL_LINE
        {"target.unit_dimension", TomlType::FloatingPoint}};

    if (auto const error_msg{ValidateConfigKeys(calibration_cfg, required_keys, optional_keys, false)}) {
        return error_msg;  // LCOV_EXCL_LINE
    }

    return std::nullopt;
}

}  // namespace reprojection::config
