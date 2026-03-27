#pragma once

#include <algorithm>
#include <filesystem>
#include <optional>
#include <string>
#include <variant>

#include <toml++/toml.hpp>

#include "types/config.hpp"

namespace reprojection::application {

namespace fs = std::filesystem;

// Adopted from https://stackoverflow.com/questions/865668/parsing-command-line-arguments-in-c
inline std::optional<std::string> GetCommandOption(char const* const* const begin, char const* const* const end,
                                                   std::string const& option) {
    char const* const* itr{std::find(begin, end, option)};
    if (itr != end and ++itr != end) {
        return std::string(*itr);
    }

    return std::nullopt;
}

std::variant<toml::table, TomlErrorMsg> LoadAndValidateConfig(fs::path const& config_path);

}  // namespace reprojection::application
