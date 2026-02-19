#pragma once

#include "projection_functions/camera_model.hpp"
#include "types/calibration_types.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::testing_mocks {

// TODO(Jack): What do we do about that fact that CameraInfo and CameraMeasurements are often found together, but I do
//  not want to use the entire CalibrationDataset type because it includes the imu as well?
//
// MVG = "multiple view geometry"
std::tuple<CameraInfo, CameraMeasurements, OptimizationState> GenerateMvgData(
    int const num_samples, uint64_t const timespan_ns, CameraModel const camera_model, ArrayXd const& intrinsics,
    ImageBounds const& bounds, bool const flat = true);

// TODO(Jack): Find a better place for this function.
Isometry3d AddGaussianNoise(double const sigma_translation, double const sigma_rotation, Isometry3d pose);

}  // namespace reprojection::testing_mocks