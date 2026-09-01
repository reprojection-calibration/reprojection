#pragma once

#include <sqlite3.h>

#include <array>
#include <functional>
#include <optional>

#include <opencv2/opencv.hpp>

namespace reprojection {

// NOTE(Jack): We purposely choose this type as the public interface because it lets us use libsqlite3 as the common
// interface and lets us hide our custom database interaction logic/code entirely from the user application space.
using SqlitePtr = std::shared_ptr<sqlite3>;

// NOTE(Jack): We choose to use the cv::Mat as the datatype interface for third party applications. But technically we
// internally store the compressed .png blob to the database in the ImageLoading step before we use it, which means that
// the application might perform an unnecessary deserialization of the image just to pass it here. However, the cv::Mat
// is such a standard type that the risk of passing encoded image buffers here directly just does not make sense. If we
// benchmark it and notice a big slowdown than we can consider an optimization here.
using ImageSampler = std::function<std::optional<std::pair<uint64_t, cv::Mat>>()>;

// [omega_x, omega_y, omega_z, acc_x, acc_y, acc_z]
using ImuSampler = std::function<std::optional<std::pair<uint64_t, std::array<double, 6>>>()>;



struct ImageInput {
    ImageSampler source;
    std::string signature;
};

// A map of image inputs indexed but their sensor_name.
// TODO(Jack): Wish there was a structural way to ensure that the map keys here were the sensor names as taken from the
// parsed calibration config.
using ImageInputs = std::map<std::string, ImageInput>;

// TODO(Jack): Once we get the app running in unit testing with test imu data we can remove this coverage exclusion!
struct ImuInput {  // LCOV_EXCL_LINE
    ImuSampler source;
    std::string signature;
};

}  // namespace reprojection
