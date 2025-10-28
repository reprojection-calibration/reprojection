#pragma once

#include <memory>

#include "projection_functions/camera_model.hpp"
#include "spline/se3_spline.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::testing_mocks {

struct MvgFrame {
    Eigen::Isometry3d pose;
    Eigen::MatrixX2d pixels;
    Eigen::MatrixX3d points;
};

// MVG = "multiple view geometry"
class MvgGenerator {
   public:
    explicit MvgGenerator(std::unique_ptr<projection_functions::Camera> const camera, bool const flat = true);

    // Input is fractional time of trajectory from [0,1)
    MvgFrame Generate(double const t) const;

    Array4d GetK() const;

    static Eigen::MatrixX2d Project(Eigen::MatrixX3d const& points_w,
                                    std::unique_ptr<projection_functions::Camera> const& camera,
                                    Eigen::Isometry3d const& tf_co_w);

   private:
    static Eigen::MatrixX3d BuildTargetPoints(bool const flat);

    std::unique_ptr<projection_functions::Camera> camera_;
    spline::Se3Spline se3_spline_;
    Eigen::MatrixX3d points_;
};

}  // namespace reprojection::testing_mocks