#pragma once

#include <optional>
#include <string>

namespace reprojection::application {

// Adopted from https://stackoverflow.com/questions/865668/parsing-command-line-arguments-in-c
std::optional<std::string> GetCommandOption(char const* const* const begin, char const* const* const end,
                                            std::string const& option);

}  // namespace reprojection::application
