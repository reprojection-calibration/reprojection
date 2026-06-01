#pragma once

#include "spline/se3_spline.hpp"
#include "types/calibration_types.hpp"
#include "types/ceres_types.hpp"

namespace reprojection::optimization {

// TODO(Jack): This should also return the poses from the spline evaluation at each target location.
ReprojectionErrors ReprojectionErrorSpline(CameraInfo const& sensor, CameraMeasurements const& targets,
                                           CameraState const& camera_state, spline::Se3Spline const& spline);

void ImuError(ImuMeasurements const& imu_data, Array6d const& tf_imu_co, Array3d const& gravity_w,
              spline::Se3Spline const& spline);

}  // namespace  reprojection::optimization
