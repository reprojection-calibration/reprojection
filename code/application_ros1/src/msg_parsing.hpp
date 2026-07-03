#pragma once

#include <rosbag/message_instance.h>

#include <array>

#include <opencv2/core/mat.hpp>

namespace reprojection::ros1 {

std::pair<uint64_t, cv::Mat> ToCvMat(rosbag::MessageInstance const& msg);

std::pair<uint64_t, std::array<double, 6>> ToImuArray(rosbag::MessageInstance const& msg);

}  // namespace reprojection::ros1
