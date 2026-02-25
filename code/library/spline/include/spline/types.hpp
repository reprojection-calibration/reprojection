#pragma once

#include "spline/constants.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::spline {

// NOTE(Jack): We need to provide these templated types so that we can also use them for the parts where we need to
// handle ceres autodiff Jet types too.
template <typename T>
using MatrixNK = Eigen::Matrix<T, constants::states, constants::order>;

using MatrixNKd = MatrixNK<double>;
using MatrixKd = Eigen::Matrix<double, constants::order, constants::order>;
using VectorKd = Eigen::Vector<double, constants::order>;

// For both the r3 and so3 spline we can evaluate either the position, velocity, or acceleration.
enum class DerivativeOrder { Null = 0, First = 1, Second = 2 };

}  // namespace reprojection::spline