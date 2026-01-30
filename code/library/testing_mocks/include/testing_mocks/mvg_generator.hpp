#pragma once

#include <memory>

#include "projection_functions/camera_model.hpp"
#include "types/calibration_types.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::testing_mocks {

CameraCalibrationData GenerateMvgData(int const num_frames, CameraModel const camera_model, ArrayXd const& intrinsics,
                                      ImageBounds const& bounds, bool const flat = true);

// TODO(Jack): Refactor the class to explicitly generate targets, there is no reason to pretend it also needs to do
//  3D random points.
// TODO(Jack): Struct naming.
//
// MVG = "multiple view geometry"
struct MvgHelpers {
    static std::tuple<MatrixX2d, ArrayXb> Project(MatrixX3d const& points_w,
                                                  std::unique_ptr<projection_functions::Camera> const& camera,
                                                  Isometry3d const& tf_co_w);

    static MatrixX3d BuildTargetPoints(bool const flat);
};

Isometry3d AddGaussianNoise(double const sigma_translation, double const sigma_rotation, Isometry3d pose);

}  // namespace reprojection::testing_mocks