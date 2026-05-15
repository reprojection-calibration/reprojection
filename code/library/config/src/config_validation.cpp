#include "config/config_validation.hpp"

#include <toml++/toml.hpp>

#include "toml_helpers.hpp"

namespace reprojection::config {

std::optional<TomlErrorMsg> ValidateCalibrationConfig(toml::table const& calibration_cfg) {
    std::map<std::string, TomlType> const required_keys{{"sensor", TomlType::Table},
                                                        {"sensor.camera_model", TomlType::String},
                                                        {"sensor.camera_name", TomlType::String},
                                                        {"target", TomlType::Table},
                                                        {"target.pattern_size", TomlType::Array},
                                                        {"target.type", TomlType::String}};

    std::map<std::string, TomlType> const optional_keys{{"application", TomlType::Table},
                                                        {"application.show_extraction", TomlType::Boolean},
                                                        {"target.circle_grid", TomlType::Table},
                                                        {"target.circle_grid.asymmetric", TomlType::Boolean},
                                                        {"target.unit_dimension", TomlType::FloatingPoint}};

    if (auto const error_msg{ValidateConfigKeys(calibration_cfg, required_keys, optional_keys, false)}) {
        return error_msg;  // LCOV_EXCL_LINE
    }

    return std::nullopt;
}

}  // namespace reprojection::config
