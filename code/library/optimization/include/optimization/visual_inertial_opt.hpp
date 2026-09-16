#pragma once

#include "optimization/bundle_adjustment.hpp"
#include "spline/se3_spline.hpp"
#include "types/calibration_types.hpp"
#include "types/ceres_types.hpp"
#include "types/transform_types.hpp"

namespace reprojection::optimization {

// TODO(Jack): This has way too many arguments... is that just how it is? Or a sign that we are missing a clean
// abstraction?
std::tuple<spline::Se3Spline, Array6d, Vector3d, CeresState> VisualInertialOpt(
    ImuSamples const& imu_data, spline::Se3Spline spline, Array6d se3_imu_rig, Vector3d gravity,
    CameraInfo const& sensor, TargetSamples const& targets, Intrinsic const& intrinsic, int num_threads);

BundleAdjustment::Problem SingleSplineCamProblem(CameraInfo const& camera_info, Intrinsic const& intrinsic,
                                                 TargetSamples const& targets, spline::Se3Spline const& spline_w_co,
                                                 AssetId camera_id);

ImuErrors EvaluateImuError(ImuSamples const& imu_data, Extrinsic const& extrinsic, Vector3d const& gravity,
                           spline::Se3Spline const& spline_w_co);

}  // namespace  reprojection::optimization
