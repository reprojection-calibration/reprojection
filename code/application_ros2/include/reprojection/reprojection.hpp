#pragma once

#include <optional>
#include <string>

#include <rosbag2_cpp/reader.hpp>

namespace reprojection::ros2 {

std::optional<std::string> SerializeBagTopic(std::string_view bag, std::string_view topic);

}  // namespace reprojection::ros2