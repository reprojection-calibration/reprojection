#pragma once

#include <Eigen/Dense>

namespace reprojection::pnp {

// MVG = "multiple view geometry"
struct MvgFrame {
    Eigen::Vector<double, 6> pose;
    Eigen::MatrixX2d pixels;
    Eigen::MatrixX3d points;
};

class MvgFrameGenerator {
   public:
    MvgFrameGenerator(bool const flat = false,
                      Eigen::Matrix3d const& K = Eigen::Matrix3d{{600, 0, 360}, {0, 600, 240}, {0, 0, 1}});

    MvgFrame Generate() const;

    Eigen::Matrix3d GetK() const;

    // Public only for testing
    static Eigen::Vector3d TrackPoint(Eigen::Vector3d const& point, Eigen::Vector3d const& position);

    // Public only for testing
    static Eigen::MatrixX2d Project(Eigen::MatrixX3d const& points_w, Eigen::Matrix3d const& K,
                                    Eigen::Isometry3d const& tf_co_w);

   private:
    Eigen::MatrixX3d points_;  // TODO(Jack): Should we add the w postfix to specify the coordinate frame?
    Eigen::Matrix3d K_;
};

}  // namespace reprojection::pnp
