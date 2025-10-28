#pragma once

#include "projection_functions/pinhole.hpp"
#include "projection_functions/pinhole_radtan4.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::projection_functions {

// TODO(Jack): Consider, if for a first show we actually want to use these classes for the optimization that we can
// template the type so ceres jet can use it, instead of hardcoding double for Project and Unproject

class Camera {
   public:
    virtual ~Camera() = default;

    virtual MatrixX2d Project(MatrixX3d const& points_co) const = 0;

    virtual MatrixX3d Unproject(MatrixX2d const& pixels) const = 0;
};

template <typename Model, typename Intrinsics>
class CameraT : public Camera  {
   public:
    explicit CameraT(Intrinsics const& intrinsics) : intrinsics_{intrinsics} {}

    MatrixX2d Project(MatrixX3d const& points_co) const override {
        Eigen::MatrixX2d pixels(points_co.rows(), 2);
        for (int i{0}; i < points_co.rows(); ++i) {
            pixels.row(i) = Model::template Project<double>(intrinsics_, points_co.row(i));
        }

        return pixels;
    }

    MatrixX3d Unproject(MatrixX2d const& pixels) const override {
        Eigen::MatrixX3d rays_co(pixels.rows(), 3);
        for (int i{0}; i < pixels.rows(); ++i) {
            rays_co.row(i) = Model::template Unproject<double>(intrinsics_, pixels.row(i));
        }

        return rays_co;
    }

   private:
    Intrinsics intrinsics_;
};

using PinholeCamera = CameraT<Pinhole, Array4d>;
using PinholeRadtan4Camera = CameraT<PinholeRadtan4, Eigen::Array<double, 8, 1>>;

}  // namespace reprojection::projection_functions
