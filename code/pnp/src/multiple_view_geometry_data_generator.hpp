#pragma once

#include <Eigen/Dense>

#include "pose_utilities.hpp"

namespace reprojection_calibration::pnp {

// MVG = "multiple view geometry"
struct MvgFrame {
    Se3 pose;
    Eigen::MatrixX2d pixels;
    Eigen::MatrixX3d points;
};

class MvgFrameGenerator {
   public:
    MvgFrameGenerator();

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

}  // namespace reprojection_calibration::pnp
