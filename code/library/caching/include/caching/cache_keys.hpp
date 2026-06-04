#pragma once

#include <string>

#include "types/calibration_types.hpp"
#include "types/sensor_data_types.hpp"

namespace reprojection::caching {

std::string CacheKey(CameraInfo const& camera_info, CameraMeasurements const& camera_measurements);

std::string CacheKey(CameraInfo const& camera_info, CameraMeasurements const& camera_measurements,
                     CameraState const& camera_state);

std::string CacheKey(CameraInfo const& camera_info, CameraMeasurements const& camera_measurements,
                     CameraState const& camera_state, Eigen::Matrix<double, 6, -1> const& control_points,
                     uint64_t const t0_ns, uint64_t const delta_t_ns);

std::string CacheKey(CameraInfo const& camera_info, CameraMeasurements const& camera_measurements,
                     OptimizationState const& optimization_state);

std::string CacheKey(TargetInfo const& target_info, EncodedImages const& encoded_images, std::string_view config);

std::string CacheKey(std::string_view const& data);

std::string CacheKey(std::string_view sensor_name, CameraModel const camera_model, EncodedImages const& encoded_images);

// WARN(Jack): The spline module does a very poor job of isolating dependencies and exposing types. As I did not want to
// have the caching module depend on the spline module (very different abstraction levels), I instead decided to
// decompose the spline and pass its basic types. This is not nice because instead of using spline::MatrixNXd like we
// should for the C3 spline control points we redefine it and use Matrix3Xd here instead. It would have been nice just
// to be able to pass the C3CubicSpline here directly like we do for all the other calibration types. I think the real
// solution is to refactor the spline stuff into the generic internal types package as this is getting a little out of
// hand.
std::string CacheKey(std::string_view sensor_name, ImuMeasurements const& imu_data,
                     Eigen::Matrix<double, 6, -1> const& control_points, uint64_t const t0_ns,
                     uint64_t const delta_t_ns);

std::string CacheKey(std::string_view sensor_name, Frames const& frames);

}  // namespace reprojection::caching
