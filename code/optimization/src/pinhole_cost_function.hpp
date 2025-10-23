#pragma once

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include "projection_functions/pinhole.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::optimization {

// TODO(Jack): Can we use the se3 type here?
// TODO(Jack): Are we sure that our convention for the top three being rotation and bottom three translation is
// consistent across the project?
// NOTE(Jack): We use Eigen::Ref here so we can pass both maps (in the PinholeCostFunction.operator()) and the direct
// types (in the testing for example).
template <typename T>
Eigen::Vector<T, 3> TransformPoint(Eigen::Ref<Eigen::Vector<T, 6> const> const& tf_i_j,
                                   Eigen::Ref<Eigen::Vector<T, 3> const> const& point_j) {
    Eigen::Vector<T, 3> const axis_angle{tf_i_j.topRows(3)};
    Eigen::Vector<T, 3> point_i;
    ceres::AngleAxisRotatePoint(axis_angle.data(), point_j.data(), point_i.data());

    Eigen::Vector<T, 3> const translation{tf_i_j.bottomRows(3)};
    point_i += translation;

    return point_i;
}

// Relation between eigen and ceres: https://groups.google.com/g/ceres-solver/c/7ZH21XX6HWU
struct PinholeCostFunction {
    explicit PinholeCostFunction(Vector2d const& pixel, Vector3d const& point) : pixel_{pixel}, point_{point} {}

    // This is the contact line were ceres requirements for using raw pointers hits our desire to use more expressive
    // eigen types. That is why here in the operator() we find the usage of the Eigen::Map class.
    template <typename T>
    bool operator()(T const* const pinhole_intrinsics, T const* const input_pose, T* const residual) const {
        // WARN(Jack): Is there a way to canonically force the program to make sure the right memory is allocated and
        // referenced to by these raw pointers? Ceres forces this pointer interface so I don't think we can easily do
        // that but consider how we  can design the program to handle that automatically.
        Eigen::Map<Eigen::Vector<T, 6> const> pose(input_pose);
        Eigen::Vector<T, 3> const point_co{TransformPoint<T>(pose, point_.cast<T>())};

        Eigen::Vector<T, 2> const pixel{projection_functions::PinholeProjection(pinhole_intrinsics, point_co)};

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
