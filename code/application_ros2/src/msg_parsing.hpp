#pragma once

#include <opencv2/core/mat.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>

namespace reprojection::ros2 {

std::pair<uint64_t, cv::Mat> ToCvMat(rosbag2_storage::SerializedBagMessage const& bag_msg, std::string_view type);

std::pair<uint64_t, std::array<double, 6>> ToImuArray(rosbag2_storage::SerializedBagMessage const& bag_msg);

}  // namespace reprojection::ros2
