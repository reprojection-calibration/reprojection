#pragma once

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <Eigen/Core>

namespace reprojection::optimization {

// TODO(Jack): Can we use the se3 type here?
// TODO(Jack): Are we sure that our convention for the top three being rotation and bottom three translation is
// consistent across the project?
// NOTE(Jack): We use Eigen::Ref here so we can pass both maps (in the PinholeCostFunction.operator()) and the direct
// types (in the testing for example).
// TODO(Jack): Does the point here really need to be templated? Or as a constant can we avoid that?
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

}  // namespace  reprojection::optimization
