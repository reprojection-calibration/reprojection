#pragma once

#include <rosbag/message_instance.h>

#include <utility>

#include <opencv2/core/mat.hpp>

namespace reprojection::ros1 {

std::pair<uint64_t, cv::Mat> ToCvMat(rosbag::MessageInstance const& msg);

}  // namespace reprojection::ros1
