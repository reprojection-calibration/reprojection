#pragma once

#include <sqlite3.h>

#include <cstdint>
#include <memory>
#include <opencv2/opencv.hpp>
#include <optional>
#include <set>
#include <string>

#include "database/calibration_database.hpp"

namespace reprojection::database {

struct ImuData {
    uint64_t timestamp_ns;

    double angular_velocity[3];
    double linear_acceleration[3];
};

struct ImageData {
    uint64_t timestamp_ns;

    cv::Mat image;
};

// TODO(Jack): Template so it also works with image data?
bool operator<(ImuData const& x, ImuData const& y) { return x.timestamp_ns < y.timestamp_ns; }

[[nodiscard]] bool AddImuData(std::string const& sensor_name, ImuData const& data,
                              std::shared_ptr<CalibrationDatabase> const database);

std::optional<std::set<ImuData>> GetImuData(std::string const& sensor_name,
                                            std::shared_ptr<CalibrationDatabase const> const database);

[[nodiscard]] bool AddImage(std::string const& sensor_name, ImageData const& data,
                            std::shared_ptr<CalibrationDatabase> const database);

}  // namespace reprojection::database