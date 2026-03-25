#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <variant>

namespace reprojection::application {

namespace fs = std::filesystem;

struct PathConfig {
    fs::path config_path;
    fs::path data_path;
    fs::path workspace_dir;
};

struct CliErrorMsg {
    std::string msg;
};

std::variant<PathConfig, CliErrorMsg> ParseCommandLineInput(int const argc, char const* const argv[]);

// Adopted from https://stackoverflow.com/questions/865668/parsing-command-line-arguments-in-c
std::optional<std::string> GetCommandOption(char const* const* const begin, char const* const* const end,
                                            std::string const& option);

}  // namespace reprojection::application
