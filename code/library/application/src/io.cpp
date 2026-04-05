#include "application/io.hpp"

#include "config/config_loading.hpp"
#include "config/config_validation.hpp"
#include "database/calibration_database.hpp"
#include "logging/logging.hpp"

namespace reprojection::application {

namespace {

auto const log{logging::Get("application")};

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

    log->info("{{'config_path': '{}', 'data_path': '{}', 'workspace_dir': '{}'}}", path_config.config_path.string(),
              path_config.data_path.string(), path_config.workspace_dir.string());

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

std::optional<toml::table> LoadAndValidateConfig(fs::path const& config_path) {
    auto const loaded_config{config::LoadConfigFile(config_path)};
    if (std::holds_alternative<TomlErrorMsg>(loaded_config)) {
        auto const error_msg{std::get<TomlErrorMsg>(loaded_config)};
        log->error("{{'toml_error': '{}', 'message': '{}'}}", ToString(error_msg.error), error_msg.msg);
        return std::nullopt;
    }

    if (auto const error_msg{config::ValidateCalibrationConfig(std::get<toml::table>(loaded_config))}) {
        log->error("{{'toml_error': '{}', 'message': '{}'}}", ToString(error_msg->error), error_msg->msg);
        return std::nullopt;
    }

    auto const config{std::get<toml::table>(loaded_config)};
    log->info("{{'config_path': '{}', 'config': {}}}", config_path.string(), logging::ToOneLineJson(config));

    return config;
}

std::optional<database::DbPtr> Open(fs::path const& workspace_dir, fs::path const& data_path) {
    if (std::error_code code; not fs::is_directory(workspace_dir, code)) {
        log->error("{{'workspace_dir': '{}', 'error_code': {{'value': {}, 'message': '{}'}}}}", workspace_dir.string(),
                   code.value(), code.message());
        return std::nullopt;
    } else if (not fs::exists(data_path, code)) {
        log->error("{{'data_path': '{}', 'error_code': {{'value': {}, 'message': '{}'}}}}", workspace_dir.string(),
                   code.value(), code.message());
        return std::nullopt;
    }

    // Handle the case that the input data is a file (ex. ROS1 bag) or a folder (ex. ROS2 bag).
    std::string const data_name{fs::is_regular_file(data_path) ? data_path.stem().string()
                                                               : data_path.parent_path().filename().string()};

    // If the database already exists then open it, if it does not exist then create and open it.
    fs::path const db_path{workspace_dir / (data_name + ".db3")};
    if (fs::exists(db_path)) {
        return std::make_shared<database::CalibrationDatabase>(db_path, false, false);
    }

    return std::make_shared<database::CalibrationDatabase>(db_path, true, false);
}

}  // namespace reprojection::application
