#pragma once

#include <memory>
#include <vector>

#include "projection_functions/camera_model.hpp"
// TODO(Jack): Remove the spline dependency from this file as it forces a dependency on the spline package for all
// consumers of the testing mocks! Could we maybe just make the spline a static variable?
#include "spline/se3_spline.hpp"
#include "types/algorithm_types.hpp"
#include "types/calibration_types.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::testing_mocks {

CameraCalibrationData GenerateMvgData(int const num_frames, CameraModel const camera_model, ArrayXd const& intrinsics,
                                      ImageBounds const& bounds, bool const flat = true);

// TODO(Jack): Why is this even a class if at almost all places we call GenerateBatch() and then are done with it?
// I think this can probably be refactored into a single function call instead.
// TODO(Jack): Refactor the class to explicitly generate targets, there is no reason to pretend it also needs to do
// 3D random points. MVG = "multiple view geometry"
class MvgGenerator {
   public:
    explicit MvgGenerator(CameraModel const camera_model, ArrayXd const& intrinsics, ImageBounds const& bounds,
                          bool const flat = true);

    CameraCalibrationData GenerateBatch(int const num_frames) const;

    static std::tuple<MatrixX2d, ArrayXb> Project(MatrixX3d const& points_w,
                                                  std::unique_ptr<projection_functions::Camera> const& camera,
                                                  Isometry3d const& tf_co_w);

    static MatrixX3d BuildTargetPoints(bool const flat);

   private:
    // Input is fractional time of trajectory from [0,1)
    std::tuple<Bundle, Isometry3d> Generate(double const t) const;

    CameraModel camera_model_;
    ArrayXd intrinsics_;
    ImageBounds bounds_;
    std::unique_ptr<projection_functions::Camera> camera_;
    spline::Se3Spline se3_spline_;
    MatrixX3d points_;
};

Isometry3d AddGaussianNoise(double const sigma_translation, double const sigma_rotation, Isometry3d pose);

}  // namespace reprojection::testing_mocks