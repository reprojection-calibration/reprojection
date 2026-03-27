#include "application/io.hpp"

#include "config/config_loading.hpp"
#include "config/config_validation.hpp"

#include "logging.hpp"

namespace reprojection::application {

namespace {

auto const log{logging::get("application")};

}

std::optional<PathConfig> ParseCommandLineInput(int const argc, char const* const argv[]) {
    auto const config_path{GetCommandOption(argv, argv + argc, "--config")};
    if (not config_path) {
        log->error("Missing --config flag");
        return std::nullopt;
    }

    auto const data_path{GetCommandOption(argv, argv + argc, "--data")};
    if (not data_path) {
        log->error("Missing --data flag");
        return std::nullopt;
    }

    PathConfig path_config{*config_path, *data_path, ""};

    // If a workspace directory is provided use it, otherwise just use the directory where the data is.
    auto const workspace_dir{GetCommandOption(argv, argv + argc, "--workspace")};
    path_config.workspace_dir = workspace_dir ? fs::path{*workspace_dir} : path_config.data_path.parent_path();

    return path_config;
}

std::optional<std::string> GetCommandOption(char const* const* const begin, char const* const* const end,
                                            std::string const& option) {
    char const* const* itr{std::find(begin, end, option)};
    if (itr != end and ++itr != end) {
        return std::string(*itr);
    }

    return std::nullopt;
}

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
