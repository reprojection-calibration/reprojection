#pragma once

#include "spline/se3_spline.hpp"
#include "types/calibration_types.hpp"
#include "types/ceres_types.hpp"

namespace reprojection::optimization {

std::pair<Frames, ReprojectionErrors> ReprojectionErrorSpline(CameraInfo const& sensor,
                                                              CameraMeasurements const& targets,
                                                              CameraState const& camera_state,
                                                              spline::Se3Spline const& spline);

ImuErrors EvaluateImuError(ImuMeasurements const& imu_data, Array6d const& tf_imu_co, Array3d const& gravity_w,
                           spline::Se3Spline const& spline);

}  // namespace  reprojection::optimization
