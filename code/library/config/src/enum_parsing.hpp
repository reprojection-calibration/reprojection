#pragma once

#include <stdexcept>
#include <string>
#include <map>

namespace reprojection::config {

template <typename T>
concept IsEnum = std::is_enum_v<T>;

template <typename T>
    requires IsEnum<T>
T ParseEnum(std::string const& parameter_name, std::map<std::string, T> const& map) {
    auto const it{map.find(parameter_name)};
    if (it == std::cend(map)) {
        // TODO(Jack): We should also print in this error message the parameters that were available in the map.
        throw std::runtime_error("Enum not found for requested parameter: " + parameter_name);
    }

    return it->second;
}

}  // namespace reprojection::config