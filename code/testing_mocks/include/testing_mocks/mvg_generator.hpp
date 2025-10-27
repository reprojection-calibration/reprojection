#pragma once

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
    explicit MvgGenerator(bool const flat = true, Array4d const& pinhole_intrinsics = Array4d{600, 600, 360, 240});

    // Input is fractional time of trajectory from [0,1)
    MvgFrame Generate(double const t) const;

    Array4d GetK() const;

    static Eigen::MatrixX2d Project(Eigen::MatrixX3d const& points_w, Array4d const& K,
                                    Eigen::Isometry3d const& tf_co_w);

   private:
    static Eigen::MatrixX3d BuildTargetPoints(bool const flat);

    Array4d pinhole_intrinsics_;
    spline::Se3Spline se3_spline_;
    Eigen::MatrixX3d points_;
};

}  // namespace reprojection::testing_mocks