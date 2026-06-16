#pragma once

#include "spline/se3_spline.hpp"
#include "types/calibration_types.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::testing_mocks {

std::pair<ImuMeasurements, spline::Se3Spline> GenerateImuData(double const duration_s, double const sample_rate_hz);

// MVG = "multiple view geometry"
std::pair<CameraMeasurements, Frames> GenerateMvgData(CameraInfo const& sensor, CameraState const& intrinsics,
                                                      double duration_s, double sample_rate_hz, bool const flat = true);

// TODO(Jack): Find a better place for this function.
Isometry3d AddGaussianNoise(double const sigma_translation, double const sigma_rotation, Isometry3d pose);

}  // namespace reprojection::testing_mocks