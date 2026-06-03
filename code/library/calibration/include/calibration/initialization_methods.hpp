#pragma once

#include "spline/spline_state.hpp"
#include "types/calibration_types.hpp"
#include "types/ceres_types.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::calibration {

/**
 * \brief Robustly initialize a camera model's intrinsic calibration using extracted target geometry constraints
 *
 * The most important contribution of this function is "robust" intrinsic initialization. It is easy to generate
 * intrinsic guesses, but it is hard to know which is the right one. This method "finds the right one" by essentially
 * running a mini calibration against a subset of the data. The intrinsic which results in the lowest final reprojection
 * error is returned.
 *
 * This is "robust" compared to other methods that either do naive averaging to the intrinsic hypothesis or only do
 * single frame reprojection error testing. Using multiple frames is the key innovation of this function.
 */
std::optional<ArrayXd> InitializeIntrinsics(CameraModel const camera_model, double const height, double const width,
                                            CameraMeasurements const& targets);

Frames PoseInitialization(CameraInfo const& sensor, CameraMeasurements const& targets, CameraState const& intrinsics);

// TODO(Jack): This point applies much more to the spline package, but it comes to the surface here. And that is that
//  here the spline we are passing HAS to be a orientation spline, but that is not coded into the type system. Instead
//  we have the generic CubicBSplineC3 type, and their is no structural guarantee or encouragement about what is
//  actually inside of it. This is something which plagues the entire spline code, not a deal breaker, but an idea that
//  smells to me.
std::pair<std::pair<Array3d, CeresState>, Vector3d> EstimateCameraImuAlignment(
    spline::CubicBSplineC3 const& camera_orientation, ImuMeasurements const& imu_data);

}  // namespace reprojection::calibration
