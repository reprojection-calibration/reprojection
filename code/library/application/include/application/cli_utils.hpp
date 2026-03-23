#pragma once

#include <algorithm>
#include <optional>
#include <string>

namespace reprojection::application {

// Adopted from https://stackoverflow.com/questions/865668/parsing-command-line-arguments-in-c
inline std::optional<std::string> GetCommandOption(char const* const* const begin, char const* const* const end,
                                                   std::string const& option) {
    char const* const* itr = std::find(begin, end, option);
    if (itr != end && ++itr != end) {
        return *itr;
    }

    return std::nullopt;
}

}  // namespace reprojection::application
