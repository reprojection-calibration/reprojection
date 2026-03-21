#pragma once

#include <string>
#include <tuple>

#include "reprojection/bag_wrapper.hpp"

namespace reprojection::ros1 {

std::tuple<std::string, std::string> DummyLoadConfig();

std::optional<std::string> CalculateCacheString(BagWrapper const& bag, std::string_view topic);

}  // namespace reprojection::ros1