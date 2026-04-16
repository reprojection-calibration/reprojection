#pragma once

#include <string>

#include "types/calibration_types.hpp"
#include "types/sensor_data_types.hpp"

namespace reprojection::caching {

std::string CacheKey(std::string_view const& data);

std::string CacheKey(std::string_view config, EncodedImages const& encoded_images);

std::string CacheKey(CameraInfo const& camera_info, CameraMeasurements const& camera_measurements);

std::string CacheKey(CameraInfo const& camera_info, CameraMeasurements const& camera_measurements,
                     CameraState const& camera_state);

std::string CacheKey(CameraInfo const& camera_info, CameraMeasurements const& camera_measurements,
                     OptimizationState const& optimization_state);

}  // namespace reprojection::caching
