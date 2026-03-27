#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <variant>

#include <toml++/toml.hpp>

#include "application/calibration_database_forward_declaration.hpp"
#include "types/config.hpp"

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

namespace fs = std::filesystem;
using DbPtr = std::shared_ptr<database::CalibrationDatabase>;

// TODO NAMING!
struct DbErrorMsg {
    std::string msg;
};

// TODO(Jack): We probably want to tell the user that the database was opened and/or if it was created new or we are
//  using an old one etc.
std::variant<DbPtr, DbErrorMsg> Open(fs::path const& workspace, fs::path const& data_source);

// NOTE(Jack): We purposely chose the toml::table as the primary configuration interface for use with the applications.
// The reason for this is that the toml++ library is pretty generic and can be installed anywhere without much effort.
// Using it allows us to hide our types internally. Imagine if we parsed and handled the entire config on the
// application side, then we would need to include a ceres solver header on the application side... or any other random
// internal type that we have. That would be ridiculous! Yes, using the toml::table means that we do not have the type
// system to save us, but we have a robust validation logic which for dynamic config handling like we do here gets the
// job done!
std::variant<toml::table, TomlErrorMsg> LoadAndValidateConfig(fs::path const& config_path);

}  // namespace reprojection::application
