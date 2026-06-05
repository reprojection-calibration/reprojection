#pragma once

#include "spline/se3_spline.hpp"
#include "types/calibration_types.hpp"
#include "types/ceres_types.hpp"

namespace reprojection::optimization {

// TODO(Jack): This has SOOOO many arguments... is that just how it is? Or a sign that we are missing a clean
// abstraction?
// TODO(Jack): Get a type for the extrinsic calibration! Both the tf and gravity!
// TODO(Jack): Return ceres state diagnostics?
std::tuple<spline::Se3Spline, Array6d, Array3d, CeresState> ExtrinsicOptimization(
    ImuMeasurements const& imu_data, spline::Se3Spline const& initial_spline, Array6d const& initial_tf_imu_co,
    Array3d const& initial_gravity_w, CameraInfo const& sensor, CameraMeasurements const& targets,
    CameraState const& intrinsics);

std::pair<Frames, ReprojectionErrors> ReprojectionErrorSpline(spline::Se3Spline const& spline, CameraInfo const& sensor,
                                                              CameraMeasurements const& targets,
                                                              CameraState const& intrinsics);

ImuErrors EvaluateImuError(ImuMeasurements const& imu_data, spline::Se3Spline const& spline, Array6d const& tf_imu_co,
                           Array3d const& gravity_w);

}  // namespace  reprojection::optimization
