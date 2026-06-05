#pragma once

#include "spline/se3_spline.hpp"
#include "types/calibration_types.hpp"
#include "types/ceres_types.hpp"

namespace reprojection::optimization {

// TODO(Jack): This has SOOOO many arguments... is that just how it is? Or a sign that we are missing a clean
// abstraction?
std::tuple<> ExtrinsicOptimization(ImuMeasurements const& imu_data, spline::Se3Spline const& spline,
                                   Array6d const& tf_imu_co, Array3d const& gravity_w, CameraInfo const& sensor,
                                   CameraMeasurements const& targets, CameraState const& intrinsics);

std::pair<Frames, ReprojectionErrors> ReprojectionErrorSpline(spline::Se3Spline const& spline, CameraInfo const& sensor,
                                                              CameraMeasurements const& targets,
                                                              CameraState const& intrinsics);

ImuErrors EvaluateImuError(ImuMeasurements const& imu_data, spline::Se3Spline const& spline, Array6d const& tf_imu_co,
                           Array3d const& gravity_w);

}  // namespace  reprojection::optimization
