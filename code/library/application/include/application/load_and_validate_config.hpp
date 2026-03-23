#pragma once

#include <filesystem>
#include <variant>

#include <toml++/toml.hpp>

#include "types/config.hpp"

// NOTE(Jack): We purposely chose the toml::table as the primary configuration interface for use with the applications.
// The reason for this is that the toml++ library is pretty generic and can be installed anywhere without much effort.
// Using it allows us to hide our types internally. Imagine if we parsed and handled the entire config on the
// application side, then we would need to include a ceres solver header on the application side... or any other random
// internal type that we have. That would be ridiculous! Yes, using the toml::table means that we do not have the type
// system to save us, but we have a robust validation logic which for dynamic config handling like we do here gets the
// job done!

namespace reprojection::application {

namespace fs = std::filesystem;

std::variant<toml::table, TomlErrorMsg> LoadAndValidateConfig(fs::path const& config_path);

}  // namespace reprojection::application
