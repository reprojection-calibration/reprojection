#pragma once

#include "spline/constants.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::spline {

// NOTE(Jack): We need to provide these templated types so that we can also use them for the parts where we need to
// handle ceres autodiff Jet types too.
template <typename T>
using Matrix3K = Eigen::Matrix<T, 3, spline::constants::order>;

using Matrix3Kd = Matrix3K<double>;
using MatrixKd = Eigen::Matrix<double, constants::order, constants::order>;
using VectorKd = Eigen::Vector<double, constants::order>;

enum class DerivativeOrder { Null = 0, First = 1, Second = 2 };

struct C3Measurement {
    std::uint64_t t_ns;
    Vector3d r3;
    DerivativeOrder type;
};

}  // namespace reprojection::spline