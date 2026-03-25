#pragma once

#include <algorithm>
#include <filesystem>
#include <optional>
#include <string>
#include <variant>

namespace reprojection::application {

struct PathConfig {
    std::filesystem::path config_path;
    std::filesystem::path data_path;
    std::filesystem::path workspace_dir;
};

struct CliErrorMsg {
    std::string msg;
};

// Adopted from https://stackoverflow.com/questions/865668/parsing-command-line-arguments-in-c
inline std::optional<std::string> GetCommandOption(char const* const* const begin, char const* const* const end,
                                                   std::string const& option) {
    char const* const* itr{std::find(begin, end, option)};
    if (itr != end and ++itr != end) {
        return std::string(*itr);
    }

    return std::nullopt;
}

inline std::variant<PathConfig, CliErrorMsg> ParseCommandLineInput(int const argc, char const* const argv[]) {
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
    path_config.workspace_dir =
        workspace_dir ? std::filesystem::path{*workspace_dir} : path_config.data_path.parent_path();

    return path_config;
}

}  // namespace reprojection::application
