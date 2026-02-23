#pragma once

#include "spline/spline_state.hpp"
#include "types/calibration_types.hpp"
#include "types/ceres_types.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::calibration {

std::tuple<std::tuple<Matrix3d, CeresState>, Vector3d> CameraImuExtrinsicInitialization(
    Frames const& camera_poses, ImuMeasurements const& imu_data);

std::tuple<Matrix3d, CeresState> InitializeCameraImuRotation(spline::CubicBSplineC3 const& so3_spline,
                                                             ImuMeasurements const& imu_data);

Vector3d InitializeGravity(spline::CubicBSplineC3 const& so3_spline, ImuMeasurements const& imu_data,
                           Matrix3d const& R_imu_co);

}  // namespace reprojection::calibration
