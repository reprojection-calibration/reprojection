#pragma once

#include <functional>
#include <optional>

#include <opencv2/opencv.hpp>

namespace reprojection {

using ImageSource = std::function<std::optional<std::pair<uint64_t, cv::Mat>>()>;

}  // namespace reprojection
