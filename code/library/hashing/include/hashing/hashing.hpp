#pragma once

#include <string>

#include "hashing/serialize.hpp"

namespace reprojection::hashing {

std::string Sha256(std::string_view input);

// NOTE(Jack): A helper function which calls the serialize method on every argument passed - this requires that every
// argument passed here has to have a fitting Serialize() function defined for it.
template <typename... Args>
std::string HashArguments(Args const&... args) {
    std::string data;
    (data.append(Serialize(args)), ...);

    return Sha256(data);
}

}  // namespace reprojection::hashing
