#pragma once

#include <optional>
#include <string>

#include <rosbag2_cpp/reader.hpp>

namespace reprojection::ros2 {

// TODO/WARN(Jack): Do the pass by value semantics of the bag here make sense? Or should we just pass the path and open
// it here new?
std::optional<std::string> SerializeBagTopic(std::string_view bag, std::string_view topic);

}  // namespace reprojection::ros2