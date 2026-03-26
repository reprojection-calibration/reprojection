#pragma once

#include <rosbag/message_instance.h>

#include <opencv2/core/mat.hpp>
#include <utility>

namespace reprojection::ros1 {

std::pair<uint64_t, cv::Mat> ToCvMat(rosbag::MessageInstance const& msg);

}  // namespace reprojection::ros1
