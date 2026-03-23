#include "application/load_and_validate_config.hpp"

#include "config/config_loading.hpp"
#include "config/config_validation.hpp"

namespace reprojection::application {

std::variant<toml::table, TomlErrorMsg> LoadAndValidateConfig(fs::path const& config_path) {
    auto const loaded_config{config::LoadConfigFile(config_path)};
    if (std::holds_alternative<TomlErrorMsg>(loaded_config)) {
        return std::get<TomlErrorMsg>(loaded_config);
    }

    if (auto const error_msg{config::ValidateCalibrationConfig(std::get<toml::table>(loaded_config))}) {
        return *error_msg;
    }

    return std::get<toml::table>(loaded_config);
}

}  // namespace reprojection::application
