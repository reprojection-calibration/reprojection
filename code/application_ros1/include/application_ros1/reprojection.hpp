#pragma once

#include <optional>
#include <string>

#include <opencv2/opencv.hpp>

#include "application_ros1/bag_wrapper.hpp"

namespace reprojection::ros1 {

std::optional<std::string> SerializeBagTopic(SingleTopicBagReader const& data);

class ImageSource {
   public:
    explicit ImageSource(SingleTopicBagReader const& reader);

    std::optional<std::pair<uint64_t, cv::Mat>> operator()();

   private:
    rosbag::View::iterator itr_;
    rosbag::View::iterator end_;
};

}  // namespace reprojection::ros1