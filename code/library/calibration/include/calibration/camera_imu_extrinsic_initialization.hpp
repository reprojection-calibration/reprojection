#pragma once

#include "spline/spline_state.hpp"
#include "types/calibration_types.hpp"
#include "types/ceres_types.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::calibration {

std::tuple<std::tuple<Matrix3d, CeresState>, Vector3d> EstimateCameraImuRotationAndGravity(
    Frames const& camera_poses, ImuMeasurements const& imu_data);

// TODO(Jack): Unit test
std::tuple<Matrix3d, CeresState> EstimateCameraImuRotation(spline::CubicBSplineC3 const& camera_orientation_spline,
                                                           ImuMeasurements const& imu_data);

// TODO(Jack): Unit test
Vector3d EstimateGravity(spline::CubicBSplineC3 const& camera_orientation_spline, ImuMeasurements const& imu_data,
                         Matrix3d const& R_imu_co);

}  // namespace reprojection::calibration
