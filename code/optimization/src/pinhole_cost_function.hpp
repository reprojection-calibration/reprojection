#pragma once

#include "ceres_xxx.hpp"
#include "projection_functions/pinhole.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::optimization {

// Relation between eigen and ceres: https://groups.google.com/g/ceres-solver/c/7ZH21XX6HWU
struct PinholeCostFunction {
    explicit PinholeCostFunction(Vector2d const& pixel, Vector3d const& point) : pixel_{pixel}, point_{point} {}

    // This is the contact line were ceres requirements for using raw pointers hits our desire to use more expressive
    // eigen types. That is why here in the operator() we find the usage of the Eigen::Map class.
    template <typename T>
    bool operator()(T const* const intrinsics_ptr, T const* const pose_ptr, T* const residual) const {
        // WARN(Jack): Is there a way to canonically force the program to make sure the right memory is allocated and
        // referenced to by these raw pointers? Ceres forces this pointer interface so I don't think we can easily do
        // that but consider how we  can design the program to handle that automatically.
        Eigen::Map<Eigen::Vector<T, 6> const> pose(pose_ptr);
        Eigen::Vector<T, 3> const point_co{TransformPoint<T>(pose, point_.cast<T>())};

        Eigen::Map<Eigen::Array<T, 4, 1> const> intrinsics(intrinsics_ptr);
        Eigen::Vector<T, 2> const pixel{projection_functions::PinholeProjection<T>(intrinsics, point_co)};

        residual[0] = T(pixel_[0]) - pixel[0];
        residual[1] = T(pixel_[1]) - pixel[1];

        return true;
    }

    static ceres::CostFunction* Create(Vector2d const& pixel, Vector3d const& point) {
        return new ceres::AutoDiffCostFunction<PinholeCostFunction, 2, 4, 6>(new PinholeCostFunction(pixel, point));
    }

    Vector2d pixel_;
    Vector3d point_;
};

}  // namespace  reprojection::optimization
