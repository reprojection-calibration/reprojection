#pragma once

#include <Eigen/Dense>

#include "constants.hpp"

namespace reprojection::spline {

using MatrixDK = Eigen::Matrix<double, constants::d, constants::k>;
using MatrixKK = Eigen::Matrix<double, constants::k, constants::k>;
using VectorD = Eigen::Vector<double, constants::d>;
using VectorK = Eigen::Vector<double, constants::k>;

enum class DerivativeOrder { Null = 0, First = 1, Second = 2 };

}  // namespace reprojection::spline