#pragma once

#include <string>
#include <tuple>

#include "reprojection/bag_wrapper.hpp"

namespace reprojection::ros1 {

std::optional<std::string> TopicCacheString(BagWrapper const& bag, std::string_view topic);

}  // namespace reprojection::ros1