#include "application/cli_utils.hpp"

#include <algorithm>
#include <optional>
#include <ranges>
#include <string>

namespace reprojection::application {

std::optional<std::string> GetCommandOption(char const* const* const begin, char const* const* const end,
                                            std::string const& option) {
    if (not begin or not end) {
        return std::nullopt;
    }

    char const* const* itr{std::find(begin, end, option)};
    if (itr != end and ++itr != end) {
        return std::string(*itr);
    }

    return std::nullopt;
}

}  // namespace reprojection::application
