#pragma once

#include <optional>
#include <string>

#include "reprojection/bag_wrapper.hpp"

namespace reprojection::ros1 {

std::optional<std::string> SerializeBagTopic(BagWrapper const& bag, std::string_view topic);

}  // namespace reprojection::ros1