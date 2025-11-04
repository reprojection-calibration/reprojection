#pragma once

#include <Eigen/Core>

#include "spline/constants.hpp"

namespace reprojection::spline {

using Matrix3K = Eigen::Matrix<double, 3, constants::k>;
using MatrixKK = Eigen::Matrix<double, constants::k, constants::k>;
using VectorK = Eigen::Vector<double, constants::k>;

enum class DerivativeOrder { Null = 0, First = 1, Second = 2 };

}  // namespace reprojection::spline