#pragma once

#include "optimization/bundle_adjustment.hpp"
#include "spline/se3_spline.hpp"
#include "types/calibration_types.hpp"
#include "types/ceres_types.hpp"
#include "types/transform_types.hpp"

namespace reprojection::optimization {

std::pair<VisualInertial::Result, CeresState> VisualInertialOpt(VisualInertial::Problem const& problem,
                                                                int num_threads);

// NOTE(Jack): We convert the continuous problem to the discrete problem so that we can use the same reprojection error
// calculation function for both cases.
Discrete::Problem ToBaProblem(VisualInertial::Problem const& problem);

ImuErrors EvaluateImuError(ImuSamples const& imu_data, Extrinsic const& extrinsic, Vector3d const& gravity,
                           spline::Se3Spline const& spline_w_co);

}  // namespace  reprojection::optimization
