#pragma once

#include <Eigen/Core>

#include "spline/constants.hpp"

namespace reprojection::spline {

// NOTE(Jack): We need to provide these templated types so that we can also use them for the parts where we need to
// handle ceres autodiff Jet types too.
template <typename T>
using Matrix3k = Eigen::Matrix<T, 3, spline::constants::order>;
template <typename T>
using VectorK = Eigen::Vector<T, constants::order>;

// TODO(Jack): Refactor names to include the double type! Also make these a specialization of the template base!
using Matrix3Kd = Matrix3k<double>;
using MatrixKK = Eigen::Matrix<double, constants::order, constants::order>;
using VectorKd = VectorK<double>;

enum class DerivativeOrder { Null = 0, First = 1, Second = 2 };

}  // namespace reprojection::spline