#pragma once

#include <optional>
#include <string>

#include <opencv2/opencv.hpp>

#include "application_ros1/bag_wrapper.hpp"

namespace reprojection::ros1 {

std::optional<std::string> SerializeBagTopic(SingleTopicBagReader const& data);

struct ImageSource {
    rosbag::View::iterator itr;
    rosbag::View::iterator end;

    std::optional<std::pair<uint64_t, cv::Mat>> operator()();
};

}  // namespace reprojection::ros1