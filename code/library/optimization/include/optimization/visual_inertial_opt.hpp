#pragma once

#include "optimization/bundle_adjustment.hpp"
#include "spline/se3_spline.hpp"
#include "types/calibration_types.hpp"
#include "types/ceres_types.hpp"
#include "types/transform_types.hpp"

namespace reprojection::optimization {

// TODO(Jack): If the targets are part of the ViProblem shouldnt the imu data also be?
std::pair<bundle_adjustment::VisualInertial::Result, CeresState> VisualInertialOpt(
    bundle_adjustment::VisualInertial::Problem const& problem, int num_threads);

// TODO(Jack): Convert directly from the continuous result/problem instead of all the input parts!
// NOTE(Jack): We convert the continious problem to the discrete problem so that we can use the same reprojection error
// calculation function for both cases.
bundle_adjustment::Discrete::Problem SingleSplineCamProblem(CameraInfo const& camera_info, Intrinsic const& intrinsic,
                                                            TargetSamples const& targets,
                                                            spline::Se3Spline const& spline_w_co, AssetId camera_id);

ImuErrors EvaluateImuError(ImuSamples const& imu_data, Extrinsic const& extrinsic, Vector3d const& gravity,
                           spline::Se3Spline const& spline_w_co);

}  // namespace  reprojection::optimization
