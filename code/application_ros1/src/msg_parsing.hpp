#pragma once

#include <rosbag/message_instance.h>

#include <opencv2/core/mat.hpp>

// TODO RENAME TOP LEVEL FILE --- NOT IMAGE SPECIFIC NOW!

namespace reprojection::ros1 {

std::pair<uint64_t, cv::Mat> ToCvMat(rosbag::MessageInstance const& msg);

std::pair<uint64_t, std::array<double, 6>> ToImuArray(rosbag::MessageInstance const& msg);

}  // namespace reprojection::ros1
