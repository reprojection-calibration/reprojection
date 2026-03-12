#pragma once

#include "spline/spline_state.hpp"
#include "types/calibration_types.hpp"
#include "types/ceres_types.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::calibration {

std::optional<ArrayXd> InitializeIntrinsics(CameraModel const camera_model, double const height, double const width,
                                            CameraMeasurements const& targets);

Frames LinearPoseInitialization(CameraInfo const& sensor, CameraMeasurements const& targets,
                                CameraState const& intrinsics);

// TODO(Jack): This point applies much more to the spline package, but it comes to the surface here. And that is that
//  here the spline we are passing HAS to be a orientation spline, but that is not coded into the type system. Instead
//  we have the generic CubicBSplineC3 type, and their is no structural guarantee or encouragement about what is
//  actually inside of it. This is something which plagues the entire spline code, not a deal breaker, but an idea that
//  smells to me.
std::tuple<std::tuple<Matrix3d, CeresState>, Vector3d> EstimateCameraImuRotationAndGravity(
    spline::CubicBSplineC3 const& camera_orientation, ImuMeasurements const& imu_data);

}  // namespace reprojection::calibration
