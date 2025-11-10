#pragma once

#include <Eigen/Core>

#include "spline/constants.hpp"

namespace reprojection::spline {

// TODO(Jack): Refactor names to include the double type! Also make these a specialization of the template base!
using Matrix3K = Eigen::Matrix<double, 3, constants::order>;
using MatrixKK = Eigen::Matrix<double, constants::order, constants::order>;
using VectorK = Eigen::Vector<double, constants::order>;

enum class DerivativeOrder { Null = 0, First = 1, Second = 2 };

}  // namespace reprojection::spline