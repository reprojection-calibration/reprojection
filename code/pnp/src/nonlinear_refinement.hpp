#pragma once

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <Eigen/Dense>

namespace reprojection::pnp {

std::tuple<Eigen::Isometry3d, Eigen::Matrix3d> NonlinearRefinement(Eigen::MatrixX2d const& pixels,
                                                                   Eigen::MatrixX3d const& points,
                                                                   Eigen::Isometry3d const& initial_pose,
                                                                   Eigen::Matrix3d const& initial_K);

// TODO(Jack): What is the final and best type for camera going to be? Raw pointer smells to me, or is at least not 100%
// necessary considering how far along with ceres we are (not far).
template <typename T>
Eigen::Vector<T, 2> PinholeProjection(T const* const camera, Eigen::Vector<T, 3> const& point) {
    T const& fx{camera[0]};
    T const& fy{camera[1]};
    T const& cx{camera[2]};
    T const& cy{camera[3]};

    T const& x{point[0]};
    T const& y{point[1]};
    T const& z{point[2]};

    // TODO(Jack): Can/should we replace this with eigen matrix operations?
    T const u{(fx * x / z) + cx};
    T const v{(fy * y / z) + cy};

    return {u, v};
}

// TODO(Jack): Can we use the se3 type here?
// NOTE(Jack): We use Eigen::Ref here so we can pass both maps (in the PinholeCostFunction.operator()) and the direct
// types (in the testing for example).
template <typename T>
Eigen::Vector<T, 3> TransformPoint(Eigen::Ref<Eigen::Vector<T, 6> const> const& tf,
                                   Eigen::Ref<Eigen::Vector<T, 3> const> const& point) {
    Eigen::Vector<T, 3> const axis_angle{tf.topRows(3)};
    Eigen::Vector<T, 3> transformed_point;
    ceres::AngleAxisRotatePoint(axis_angle.data(), point.data(), transformed_point.data());

    Eigen::Vector<T, 3> const translation{tf.bottomRows(3)};
    transformed_point += translation;

    return transformed_point;
}

// Relation between eigen and ceres: https://groups.google.com/g/ceres-solver/c/7ZH21XX6HWU
struct PinholeCostFunction {
    explicit PinholeCostFunction(Eigen::Vector2d const& pixel, Eigen::Vector3d const& point)
        : pixel_{pixel}, point_{point} {}

    // This is the contact line were ceres requirements for using raw pointers hits our desire to use more expressive
    // eigen types. That is why here in the operator() we find the usage of the Eigen::Map class.
    template <typename T>
    bool operator()(T const* const pinhole_intrinsics, T const* const input_pose, T* const residual) const {
        // WARN(Jack): Is there a way to canonically force the program to make sure the right memory is allocated and
        // referenced to by these raw pointers? Ceres forces this pointer interface so I don't think we can easily do
        // that but consider how we  can design the program to handle that automatically.
        Eigen::Map<Eigen::Vector<T, 6> const> pose(input_pose);
        Eigen::Vector<T, 3> const point_co{TransformPoint<T>(pose, point_.cast<T>())};

        Eigen::Vector<T, 2> const pixel{PinholeProjection(pinhole_intrinsics, point_co)};

        residual[0] = T(pixel_[0]) - pixel[0];
        residual[1] = T(pixel_[1]) - pixel[1];

        return true;
    }

    static ceres::CostFunction* Create(Eigen::Vector2d const& pixel, Eigen::Vector3d const& point) {
        return new ceres::AutoDiffCostFunction<PinholeCostFunction, 2, 4, 6>(new PinholeCostFunction(pixel, point));
    }

    Eigen::Vector2d pixel_;
    Eigen::Vector3d point_;
};

Eigen::Matrix3d ToK(Eigen::Array<double, 4, 1> const& array);

Eigen::Array<double, 4, 1> FromK(Eigen::Matrix3d const& matrix);

}  // namespace reprojection::pnp
