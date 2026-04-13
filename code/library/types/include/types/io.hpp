#pragma once

#include <sqlite3.h>

#include <functional>
#include <optional>

#include <opencv2/opencv.hpp>

namespace reprojection {

struct SqliteDeleter {
    void operator()(sqlite3* const db) const {
        if (db) {
            sqlite3_close(db);
        }
    }
};

using SqlitePtr = std::shared_ptr<sqlite3>;

using ImageSource = std::function<std::optional<std::pair<uint64_t, cv::Mat>>()>;

}  // namespace reprojection
