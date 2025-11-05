#pragma once

#include <memory>
#include <tuple>
#include <vector>

#include "projection_functions/camera_model.hpp"
#include "spline/se3_spline.hpp"
#include "types/calibration_types.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::testing_mocks {

struct NoiseProfile {
    double sigma_pixel;
    double sigma_point;
    double sigma_pose_translation;
    double sigma_pose_rotation;
};

// TODO(Jack): Why is this even a class if at almost all places we call GenerateBatch() and then are done with it?
// I think this can probably be refactored into a single function call instead.
// TODO(Jack): Refactor the class to explicitly generate targets, there is no reason to pretend it also needs to do 3D
// random points.
// MVG = "multiple view geometry"
class MvgGenerator {
   public:
    explicit MvgGenerator(std::unique_ptr<projection_functions::Camera> const camera, bool const flat = true);

    std::vector<Frame> GenerateBatch(int const num_frames) const;

    std::tuple<std::vector<Frame>, std::vector<Frame>> GenerateBatchWithNoise(int const num_frames,
                                                                              NoiseProfile const& sigmas) const;

    static MatrixX2d Project(MatrixX3d const& points_w, std::unique_ptr<projection_functions::Camera> const& camera,
                             Isometry3d const& tf_co_w);

   private:
    // Input is fractional time of trajectory from [0,1)
    Frame Generate(double const t) const;

    static MatrixX3d BuildTargetPoints(bool const flat);

    std::unique_ptr<projection_functions::Camera> camera_;
    spline::Se3Spline se3_spline_;
    MatrixX3d points_;
};

}  // namespace reprojection::testing_mocks