#include "config/config2.hpp"

#include "config/config_loading.hpp"
#include "config/config_validation.hpp"
#include "logging/logging.hpp"

namespace reprojection::config {

namespace {

auto const log{logging::Get("config")};

}

std::optional<Config> Config::Load(std::filesystem::path const& path) {
    auto const load_result{LoadConfigFile(path)};
    if (std::holds_alternative<TomlErrorMsg>(load_result)) {
        TomlErrorMsg const error{std::get<TomlErrorMsg>(load_result)};
        log->error("{{'toml_error': '{}', 'message': '{}'}}", ToString(error.type), error.msg);

        return std::nullopt;
    }

    toml::table const toml_table{std::get<toml::table>(load_result)};
    if (auto const error{ValidateCalibrationConfig(toml_table)}) {
        log->error("{{'toml_error': '{}', 'message': '{}'}}", ToString(error->type), error->msg);

        return std::nullopt;
    }
    auto const app_config { Config::Application::Load() }

    return;
}

}  // namespace reprojection::config