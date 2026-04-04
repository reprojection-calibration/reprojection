#pragma once

#include <optional>
#include <string>

#include <opencv2/opencv.hpp>

#include "application_ros2/bag_wrapper.hpp"

namespace reprojection::ros2 {

std::optional<std::string> SerializeBagTopic(SingleTopicBagReader const& data);

struct ImageSource {
    SingleTopicBagReader& bag_reader;

    std::optional<std::pair<uint64_t, cv::Mat>> operator()();
};

}  // namespace reprojection::ros2