#pragma once

#include <opencv2/core/mat.hpp>
#include <rosbag/message_instance.h>

namespace reprojection::ros1 {

cv::Mat ToCvMat(rosbag::MessageInstance const& msg);

}  // namespace reprojection::ros1
