#include "application/cli_utils.hpp"

#include <algorithm>

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

}  // namespace reprojection::application
