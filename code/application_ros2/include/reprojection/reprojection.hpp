#pragma once

#include <optional>
#include <string>

#include "reprojection/bag_wrapper.hpp"

namespace reprojection::ros2 {

std::optional<std::string> SerializeBagTopic(SingleTopicBagReader const& data);

}  // namespace reprojection::ros2