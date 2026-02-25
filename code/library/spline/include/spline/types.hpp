#pragma once

#include "spline/constants.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::spline {

// Holds the full se3 state which is then split up into two MatrixNXd matrices as it is passed to the separate rotation
// and translation splines.
using Matrix2NXd = Eigen::Matrix<double, 2 * constants::states, Eigen::Dynamic>;

// NOTE(Jack): We need to provide these templated types so that we can also use them for the parts where we need
// to handle ceres autodiff Jet types too.
template <typename T>
using MatrixNK = Eigen::Matrix<T, constants::states, constants::order>;
using MatrixNKd = MatrixNK<double>;

// TODO(Jack): Technically we could also just use Eigen::MatrixNXd, but I want to try to communicate that we are
//  actually working with the state here of size N, which for our case is length 3 for both splines. Maybe this is
//  dumb.
using MatrixNXd = Eigen::Matrix<double, constants::states, Eigen::Dynamic>;
using MatrixKd = Eigen::Matrix<double, constants::order, constants::order>;
using VectorKd = Eigen::Vector<double, constants::order>;

// For both the r3 and so3 spline we can evaluate either the position, velocity, or acceleration.
enum class DerivativeOrder { Null = 0, First = 1, Second = 2 };

}  // namespace reprojection::spline