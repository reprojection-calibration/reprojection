#pragma once

#include <Eigen/Dense>

#include "projection_functions/pinhole.hpp"

namespace reprojection::projection_functions {

template <typename T>
Eigen::Vector<T, 2> DoubleSphereProjection(T const* const camera, Eigen::Vector<T, 3> const& point) {
    T const& xi{camera[4]};
    T const& alpha{camera[5]};

    T const& x{point[0]};
    T const& y{point[1]};
    T const& z{point[2]};

    T const xx{x * x};
    T const yy{y * y};
    T const r2{xx + yy};
    T const d1{std::sqrt(r2 + z * z)};

    T const wz{xi * d1 + z};  // wz = "weighted z"
    T const d2{std::sqrt(r2 + wz * wz)};

    T const z_star{(alpha * d2) + (1.0 - alpha) * (xi * d1 + z)};
    Eigen::Vector<T, 3> const point_star{x, y, z_star};

    // WARN(Jack): I avoid raw pointers at all costs, so this just scares me, that we are "cleverly" passing on the
    // first four parameters of the camera from the pointer, all without bounds or validity checking! I am not sure if
    // there is an action item here, but keep you eyes peeled!
    return PinholeProjection<T>(camera, point_star);
}

}  // namespace reprojection::projection_functions