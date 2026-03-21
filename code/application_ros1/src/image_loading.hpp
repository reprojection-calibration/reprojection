#pragma once

#include <rosbag/message_instance.h>

#include <opencv2/core/mat.hpp>

namespace reprojection::ros1 {

cv::Mat ToCvMat(rosbag::MessageInstance const& msg);

}  // namespace reprojection::ros1
