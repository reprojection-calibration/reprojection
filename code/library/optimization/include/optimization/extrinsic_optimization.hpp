#pragma once

#include "spline/se3_spline.hpp"
#include "types/calibration_types.hpp"
#include "types/ceres_types.hpp"

namespace reprojection::optimization {

// TODO(Jack): This has way too many arguments... is that just how it is? Or a sign that we are missing a clean
// abstraction?
std::pair<spline::Se3Spline, ImuCamExtrinsic> ExtrinsicOptimization(
    ImuMeasurements const& imu_data, spline::Se3Spline const& initial_spline, ImuCamExtrinsic const& initial_extrinsic,
    CameraInfo const& sensor, CameraMeasurements const& targets, CameraState const& intrinsics);

std::pair<Frames, ReprojectionErrors> ReprojectionErrorSpline(CameraInfo const& sensor,
                                                              CameraMeasurements const& targets,
                                                              CameraState const& camera_state,
                                                              spline::Se3Spline const& spline);

ImuErrors EvaluateImuError(ImuMeasurements const& imu_data, ImuCamExtrinsic const& extrinsic,
                           spline::Se3Spline const& spline);

}  // namespace  reprojection::optimization
