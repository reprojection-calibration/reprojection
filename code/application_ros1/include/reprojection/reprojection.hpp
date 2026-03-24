#pragma once

#include <optional>
#include <string>

#include "reprojection/bag_wrapper.hpp"

namespace reprojection::ros1 {

// TODO(Jack): Unfortunately this cannot be const or pass by value because of the complicated bag and view semantics.
std::optional<std::string> SerializeBagTopic(SingleTopicBagReader& data);

}  // namespace reprojection::ros1