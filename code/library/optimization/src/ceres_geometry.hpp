#pragma once

#include <ceres/rotation.h>

#include <Eigen/Core>

#include "types/eigen_types.hpp"

namespace reprojection::optimization {

// TODO(Jack): Are we sure that our convention for the top three being rotation and bottom three translation is
// consistent across the project?
// NOTE(Jack): We use Eigen::Ref here so we can pass both maps (in the PinholeCostFunction.operator()) and the direct
// types (in the testing for example).
// TODO(Jack): Does the point here really need to be templated? Or as a constant can we avoid that?
template <typename T>
Vector3<T> TransformPoint(Eigen::Ref<Eigen::Vector<T, 6> const> const& tf_i_j,
                          Eigen::Ref<Vector3<T> const> const& point_j) {
    Vector3<T> const axis_angle{tf_i_j.topRows(3)};
    Vector3<T> point_i;
    ceres::AngleAxisRotatePoint(axis_angle.data(), point_j.data(), point_i.data());

    Vector3<T> const translation{tf_i_j.bottomRows(3)};
    point_i += translation;

    return point_i;
}

}  // namespace  reprojection::optimization
