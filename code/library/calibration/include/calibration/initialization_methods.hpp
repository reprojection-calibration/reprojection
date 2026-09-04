#pragma once

#include "spline/se3_spline.hpp"
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
std::optional<ArrayXd> InitializeIntrinsics(CameraModel camera_model, double height, double width,
                                            TargetSamples const& targets, int num_threads);

Frames PoseInitialization(CameraInfo const& camera_info, TargetSamples const& targets, Intrinsic const& intrinsic);

std::pair<std::pair<Array3d, CeresState>, Vector3d> EstimateCameraImuAlignment(spline::Se3Spline const& spline,
                                                                               ImuSamples const& imu_data,
                                                                               int num_threads);

}  // namespace reprojection::calibration
