#include "io.hpp"

#include "application/cli_utils.hpp"
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

    // If a workspace directory is provided use it, otherwise use the data_path. We support data path being either a
    // file or a folder - in the case of a file we use the files directory, in the case of a folder we use the folder
    // directly.
    // TODO(Jack): Clean up this crazy ternary statement used to determine the workspace directory.
    auto const workspace_dir{GetCommandOption(argv, argv + argc, "--workspace")};
    path_config.workspace_dir = workspace_dir                             ? fs::path{*workspace_dir}
                                : fs::is_directory(path_config.data_path) ? path_config.data_path
                                                                          : path_config.data_path.parent_path();

    log->info("{{'config_path': '{}', 'data_path': '{}', 'workspace_dir': '{}'}}", path_config.config_path.string(),
              path_config.data_path.string(), path_config.workspace_dir.string());

    return path_config;
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

std::optional<SqlitePtr> Open(fs::path const& workspace_dir, fs::path const& data_path) {
    if (std::error_code code; not fs::is_directory(workspace_dir, code)) {
        log->error("{{'workspace_dir': '{}', 'error_code': {{'value': {}, 'message': '{}'}}}}", workspace_dir.string(),
                   code.value(), code.message());
        return std::nullopt;
    } else if (not fs::exists(data_path, code)) {
        // TODO(Jack): The error code message does not make sense to me here. This is why I added the 'fs::exists':
        // false field. Otherwise the user would, for a non existent file or folder just see,
        //
        //      'error_code': {'value': 0, 'message': 'Success'}}
        //
        // which understandably does not communicate that this operation is messaging a failure... Not sure if I am
        // doing something wrong or just do not understand what the error code here actually represents.
        log->error("{{'data_path': '{}', 'fs::exists': false, 'error_code': {{'value': {}, 'message': '{}'}}}}",
                   data_path.string(), code.value(), code.message());
        return std::nullopt;
    }

    // Handle the case that the input data is a file (ex. ROS1 bag) or a folder (ex. ROS2 bag).
    std::string const data_name{fs::is_regular_file(data_path) ? data_path.stem().string()
                                                               : data_path.parent_path().filename().string()};

    // If the database already exists then open it, if it does not exist then create and open it.
    fs::path const db_path{workspace_dir / (data_name + ".db3")};
    bool const db_exists{fs::exists(db_path)};
    log->info("{{'db_path': '{}', 'exists': {}}}", db_path.string(), db_exists);

    if (db_exists) {
        return database::OpenCalibrationDatabase(db_path, false, false);
    }

    return database::OpenCalibrationDatabase(db_path, true, false);
}

}  // namespace reprojection::application
