#include <algorithm>

#include "application/io.hpp"
#include "config/config_loading.hpp"
#include "config/config_validation.hpp"
#include "database/calibration_database.hpp"

namespace reprojection::application {

std::variant<PathConfig, CliErrorMsg> ParseCommandLineInput(int const argc, char const* const argv[]) {
    auto const config_path{GetCommandOption(argv, argv + argc, "--config")};
    if (not config_path) {
        return CliErrorMsg{"Missing --config flag"};
    }

    auto const data_path{GetCommandOption(argv, argv + argc, "--data")};
    if (not data_path) {
        return CliErrorMsg{"Missing --data flag"};
    }

    PathConfig path_config{*config_path, *data_path, ""};

    // If a workspace directory is provided use it, otherwise just use the directory where the data is.
    auto const workspace_dir{GetCommandOption(argv, argv + argc, "--workspace")};
    path_config.workspace_dir = workspace_dir ? fs::path{*workspace_dir} : path_config.data_path.parent_path();

    return path_config;
}

// Adopted from https://stackoverflow.com/questions/865668/parsing-command-line-arguments-in-c
std::optional<std::string> GetCommandOption(char const* const* const begin, char const* const* const end,
                                            std::string const& option) {
    char const* const* itr{std::find(begin, end, option)};
    if (itr != end and ++itr != end) {
        return std::string(*itr);
    }

    return std::nullopt;
}

std::variant<DbPtr, DbErrorMsg> Open(fs::path const& workspace, fs::path const& data_source) {
    if (std::error_code code; not fs::is_directory(workspace, code)) {
        // TODO ERROR CODE PROVIDES AT LEAST STRING STREAM OPERATOR THAT DOES EXACTLY WHAT WE DO HERE AND BELOW
        //  MANUALLY!!!
        return DbErrorMsg{"Provided workspace path: '" + workspace.string() +
                          "' is not a valid directory - error code (" + std::to_string(code.value()) +
                          ") with description '" + code.message() + "'"};
    }
    if (std::error_code code; not fs::is_regular_file(data_source, code)) {
        return DbErrorMsg{"Provided data source path: '" + data_source.string() +
                          "' is not a valid file - error code (" + std::to_string(code.value()) +
                          ") with description '" + code.message() + "'"};
    }

    fs::path const db_path{workspace / (data_source.stem().string() + ".db3")};
    if (fs::exists(db_path)) {
        return std::make_shared<database::CalibrationDatabase>(db_path, false, false);
    }

    return std::make_shared<database::CalibrationDatabase>(db_path, true, false);
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
