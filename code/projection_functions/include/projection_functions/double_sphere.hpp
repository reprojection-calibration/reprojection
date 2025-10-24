#pragma once

#include <Eigen/Core>

#include "projection_functions/pinhole.hpp"

// Implemented following "The Double Sphere Camera Model" (https://arxiv.org/pdf/1807.08957)

namespace reprojection::projection_functions {

template <typename T>
Eigen::Vector<T, 2> DoubleSphereProjection(Eigen::Array<T, 6, 1> const& intrinsics, Eigen::Array<T, 3, 1> const& P_co) {
    T const& x{P_co[0]};
    T const& y{P_co[1]};
    T const& z{P_co[2]};

    T const xx{x * x};
    T const yy{y * y};
    T const r2{xx + yy};
    T const d1{std::sqrt(r2 + z * z)};

    T const& xi{intrinsics[4]};
    T const wz{xi * d1 + z};  // wz = "weighted z"
    T const d2{std::sqrt(r2 + wz * wz)};

    T const& alpha{intrinsics[5]};
    T const z_star{(alpha * d2) + (1.0 - alpha) * (xi * d1 + z)};
    Eigen::Vector<T, 3> const P_star{x, y, z_star};

    return PinholeProjection<T>(intrinsics.topRows(4), P_star);
}

}  // namespace reprojection::projection_functions