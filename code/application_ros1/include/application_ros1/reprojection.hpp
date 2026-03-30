#pragma once

#include <optional>
#include <string>

#include "application_ros1/bag_wrapper.hpp"

namespace reprojection::ros1 {

std::optional<std::string> SerializeBagTopic(SingleTopicBagReader const& data);

}  // namespace reprojection::ros1