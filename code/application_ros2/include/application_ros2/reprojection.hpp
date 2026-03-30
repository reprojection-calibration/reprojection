#pragma once

#include <optional>
#include <string>

#include "application_ros2/bag_wrapper.hpp"

namespace reprojection::ros2 {

std::optional<std::string> SerializeBagTopic(SingleTopicBagReader const& data);

}  // namespace reprojection::ros2