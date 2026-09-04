#pragma once

#include "spline/se3_spline.hpp"
#include "types/calibration_types.hpp"
#include "types/ceres_types.hpp"

namespace reprojection::optimization {

// TODO(Jack): This has way too many arguments... is that just how it is? Or a sign that we are missing a clean
// abstraction?
std::tuple<spline::Se3Spline, Extrinsic, Vector3d> ExtrinsicOptimization(
    ImuSamples const& imu_data, spline::Se3Spline const& initial_spline, Extrinsic const& initial_extrinsic,
    Vector3d const& initial_gravity, CameraInfo const& sensor, TargetSamples const& targets, Intrinsic const& intrinsic,
    int num_threads);

// RENMAE TO EvaluateResiduals
std::pair<Frames, std::vector<ReprojectionError>> ReprojectionErrorSpline(CameraInfo const& sensor,
                                                                          TargetSamples const& targets,
                                                                          Intrinsic const& intrinsic,
                                                                          spline::Se3Spline const& spline_w_co);

ImuErrors EvaluateImuError(ImuSamples const& imu_data, Extrinsic const& extrinsic, Vector3d const& gravity,
                           spline::Se3Spline const& spline_w_co);

}  // namespace  reprojection::optimization
