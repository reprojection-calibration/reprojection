#pragma once

#include <functional>

#include <opencv2/opencv.hpp>
#include <toml++/toml.hpp>

#include "application/calibration_database_forward_declaration.hpp"

namespace reprojection::application {

// TODO(Jack): Should we make a central definition of this? COPY AND PASTED
using DbPtr = std::shared_ptr<database::CalibrationDatabase>;
using ImageProvider = std::function<std::optional<std::pair<uint64_t, cv::Mat>>()>;

void Calibrate(toml::table const& config, ImageProvider image_source, std::string_view image_cache_key, DbPtr const db);

}  // namespace reprojection::application
