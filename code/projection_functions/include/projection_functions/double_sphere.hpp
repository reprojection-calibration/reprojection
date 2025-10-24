#pragma once

#include <Eigen/Core>

#include "projection_functions/pinhole.hpp"

// Implemented following "The Double Sphere Camera Model" (https://arxiv.org/pdf/1807.08957)

namespace reprojection::projection_functions {

template <typename T>
Eigen::Vector<T, 2> DoubleSphereProjection(Eigen::Array<T, 6, 1> const& intrinsics,
                                           Eigen::Array<T, 3, 1> const& point) {
    T const& x{point[0]};
    T const& y{point[1]};
    T const& z{point[2]};

    T const xx{x * x};
    T const yy{y * y};
    T const r2{xx + yy};
    T const d1{std::sqrt(r2 + z * z)};

    T const& xi{intrinsics[4]};
    T const wz{xi * d1 + z};  // wz = "weighted z"
    T const d2{std::sqrt(r2 + wz * wz)};

    T const& alpha{intrinsics[5]};
    T const z_star{(alpha * d2) + (1.0 - alpha) * (xi * d1 + z)};
    Eigen::Vector<T, 3> const point_star{x, y, z_star};

    Eigen::Array<T, 4, 1> const pinhole_intrinsics{intrinsics.topRows(4)};

    return PinholeProjection<T>(pinhole_intrinsics, point_star);
}

}  // namespace reprojection::projection_functions