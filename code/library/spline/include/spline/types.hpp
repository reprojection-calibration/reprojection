#pragma once

#include "spline/constants.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::spline {

// NOTE(Jack): Technically we could also just use Matrix6Xd (combined se3 spline) or Eigen::Matrix3Xd (so3 and r3
// splines), but I want to try to communicate that we are actually working with the state here of size N, which for our
// case is length 3 for both splines. Maybe this is dumb and I regret it one day.
// NOTE(Jack): We need to provide these templated types so that we can also use them for the parts where we need
// to handle ceres autodiff Jet types too.

static_assert(constants::states == 3, "This code was only written for a three dimensional state spline.");
static_assert(constants::order == 4, "This code was only written for a order 4 spline (cubic b-spline).");
static_assert(constants::degree == 3, "This code was only written for a cubic b-spline.");

template <typename T>
using Matrix2NK = Eigen::Matrix<T, 2 * constants::states, constants::order>;
// Matrix2NXd the full se3 state (size = 2 * constants::states) which is then split up into two MatrixNXd matrices as it
// is passed to the separate rotation and translation splines.
using Matrix2NXd = Eigen::Matrix<double, 2 * constants::states, Eigen::Dynamic>;
using MatrixKd = Eigen::Matrix<double, constants::order, constants::order>;
template <typename T>
using MatrixNK = Eigen::Matrix<T, constants::states, constants::order>;
using MatrixNKd = MatrixNK<double>;
using MatrixNXd = Eigen::Matrix<double, constants::states, Eigen::Dynamic>;

using VectorKd = Eigen::Vector<double, constants::order>;

// For both the r3 and so3 spline we can evaluate either the position, velocity, or acceleration.
enum class DerivativeOrder { Null = 0, First = 1, Second = 2 };

}  // namespace reprojection::spline