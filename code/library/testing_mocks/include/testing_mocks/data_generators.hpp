#pragma once

#include "spline/se3_spline.hpp"
#include "types/calibration_types.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::testing_mocks {

std::pair<ImuSamples, spline::Se3Spline> GenerateImuData(double duration_s, double sample_rate_hz);

// MVG = "multiple view geometry"
std::pair<TargetSamples, Frames> GenerateMvgData(CameraInfo const& sensor, Intrinsic const& intrinsics,
                                                 double duration_s, double sample_rate_hz, bool flat = true);

// TODO(Jack): Find a better place for this function.
Isometry3d AddGaussianNoise(double sigma_translation, double sigma_rotation, Isometry3d pose);

}  // namespace reprojection::testing_mocks