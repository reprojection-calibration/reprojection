#pragma once

#include "optimization/bundle_adjustment.hpp"
#include "spline/se3_spline.hpp"
#include "types/calibration_types.hpp"
#include "types/ceres_types.hpp"
#include "types/transform_types.hpp"

namespace reprojection::optimization {

// TODO(Jack): If the targets are part of the ViProblem shouldnt the imu data also be?
std::pair<BundleAdjustment::ViResult, CeresState> VisualInertialOpt(ImuSamples const& imu_data,
                                                                    BundleAdjustment::ViProblem const& problem,
                                                                    int num_threads);

// TODO(Jack): Convert directly from the ViProblem/ViResult instead of all the input parts!
BundleAdjustment::Problem SingleSplineCamProblem(CameraInfo const& camera_info, Intrinsic const& intrinsic,
                                                 TargetSamples const& targets, spline::Se3Spline const& spline_w_co,
                                                 AssetId camera_id);

ImuErrors EvaluateImuError(ImuSamples const& imu_data, Extrinsic const& extrinsic, Vector3d const& gravity,
                           spline::Se3Spline const& spline_w_co);

}  // namespace  reprojection::optimization
