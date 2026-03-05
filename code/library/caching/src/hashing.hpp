#pragma once

#include <string>
#include <string_view>

namespace reprojection::caching {

std::string Sha256(std::string_view input);

}