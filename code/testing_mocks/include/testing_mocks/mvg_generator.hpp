#pragma once

#include <Eigen/Dense>

#include "spline/se3_spline.hpp"

namespace reprojection::testing_mocks {

struct MvgFrame {
    Eigen::Isometry3d pose;
    Eigen::MatrixX2d pixels;
    Eigen::MatrixX3d points;
};

// MVG = "multiple view geometry"
class MvgGenerator {
   public:
    explicit MvgGenerator(bool const flat = true,
                          Eigen::Matrix3d const& K = Eigen::Matrix3d{{600, 0, 360}, {0, 600, 240}, {0, 0, 1}});

    // Input is fractional time of trajectory from [0,1)
    MvgFrame Generate(double const t) const;

    Eigen::Matrix3d GetK() const;

    static Eigen::MatrixX2d Project(Eigen::MatrixX3d const& points_w, Eigen::Matrix3d const& K,
                                    Eigen::Isometry3d const& tf_co_w);

   private:
    static Eigen::MatrixX3d BuildTargetPoints(bool const flat);

    Eigen::Matrix3d K_;
    spline::Se3Spline se3_spline_;
    Eigen::MatrixX3d points_;
};

}  // namespace reprojection::testing_mocks