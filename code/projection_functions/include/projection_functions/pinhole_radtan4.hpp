#pragma once

#include <Eigen/Dense>

#include "projection_functions/pinhole.hpp"

namespace reprojection::projection_functions {

template <typename T>
Eigen::Vector<T, 2> PinholeRadtan4Projection(T const* const camera, Eigen::Vector<T, 3> const& point) {
    T const& k1{camera[4]};
    T const& k2{camera[5]};
    T const& p1{camera[6]};
    T const& p2{camera[7]};

    T const& x{point[0]};
    T const& y{point[1]};
    T const& z{point[2]};

    T const x_z{x / z};
    T const y_z{y / z};
    T const x_z2{x_z * x_z};
    T const y_z2{y_z * y_z};
    T const r2{x_z2 + y_z2};
    T const r_prime{1 + (k1 * r2) + (k2 * r2 * r2)};

    T const x_star{(r_prime * x_z) + (2 * p1 * x_z * y_z) + p2 * (r2 + 2 * x_z2)};
    T const y_star{(r_prime * y_z) + (2 * p2 * x_z * y_z) + p1 * (r2 + 2 * y_z2)};

    Eigen::Vector<T, 3> const point_star{x_star, y_star, 1};  // z=1 here because we already "projected" at the start

    return PinholeProjection<T>(camera, point_star);
}

}  // namespace reprojection::projection_functions