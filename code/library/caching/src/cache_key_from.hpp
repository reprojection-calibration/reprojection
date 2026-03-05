#pragma once

#include <string>

#include "hashing.hpp"
#include "serialize.hpp"

namespace reprojection::caching {

// NOTE(Jack): A helper function which calls the serialize method on every argument passed - this requires that every
// argument passed here has to have a fitting Serialize() function defined for it!
template <typename... Args>
std::string CacheKeyFrom(Args const&... args) {
    std::string data;
    (data.append(Serialize(args)), ...);

    return Sha256(data);
}

}  // namespace reprojection::caching