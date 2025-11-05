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

    // TODO(Jack): In general the strategy of how to testy given noisy data is at this time unclear! For the non-noisy
    // case we have exact answers, but for the noisy case we will have "random" answers that are part of a distribution.
    // At this time we still have not implemented predicting those distributions, therefore we have no formal way to
    // test using noisy data. For example I think if we know there is a specific pixel noise, then there will be a
    // specific error in the poses calculated. This is something we need to look at closely!
    // NOTE(Jack): This function simply adds gaussian noise to all respective values! It does not actually add noise to
    // the points and then project those points to noisy pixels. That would be the more proper way to model that noise!
    // But as of now I cannot be sure that is specifically of interest to us, as I believe corner extractors are modeled
    // as simply adding noise in the pixel space directly. Furthermore we are not optimizing the extracted pixels or
    // points so it really probably does not matter.
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