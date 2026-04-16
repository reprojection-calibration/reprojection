#pragma once

#include <sqlite3.h>

#include <functional>
#include <optional>

#include <opencv2/opencv.hpp>

namespace reprojection {

using SqlitePtr = std::shared_ptr<sqlite3>;

// NOTE(Jack): We choose to use the cv::Mat as the datatype interface for third party applications. But technically we
// internally store the compressed .png blob to the database in the ImageLoading step before we use it, which means that
// the application might perform an unnecessary deserialization of the image just to pass it here. However, the cv::Mat
// is such a standard type that the risk of passing encoded image buffers here directly just does not make sense. If we
// benchmark it and notice a big slowdown than we can consider an optimization here.
using ImageSource = std::function<std::optional<std::pair<uint64_t, cv::Mat>>()>;

}  // namespace reprojection
