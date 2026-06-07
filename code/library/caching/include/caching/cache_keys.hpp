#pragma once

#include <string>

#include "caching/serialize.hpp"
#include "types/sensor_data_types.hpp"

namespace reprojection::caching {

std::string Sha256(std::string_view input);

// NOTE(Jack): A helper function which calls the serialize method on every argument passed - this requires that every
// argument passed here has to have a fitting Serialize() function defined for it.
template <typename... Args>
std::string CacheKeyFrom(Args const&... args) {
    std::string data;
    (data.append(Serialize(args)), ...);

    return Sha256(data);
}

}  // namespace reprojection::caching
