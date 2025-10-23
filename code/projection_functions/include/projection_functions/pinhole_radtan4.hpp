#pragma once

#include <Eigen/Core>

#include "projection_functions/pinhole.hpp"

namespace reprojection::projection_functions {

template <typename T>
Eigen::Vector<T, 2> Radtan4Distortion(Eigen::Array<T, 4, 1> const& distortion,
                                      Eigen::Array<T, 2, 1> const& projected_point) {
    T const& k1{distortion[0]};
    T const& k2{distortion[1]};
    T const& p1{distortion[2]};
    T const& p2{distortion[3]};

    T const& x_z{projected_point[0]};
    T const& y_z{projected_point[1]};

    T const x_z2{x_z * x_z};
    T const y_z2{y_z * y_z};
    T const r2{x_z2 + y_z2};

    T const r_prime{1 + (k1 * r2) + (k2 * r2 * r2)};
    T const x_star{(r_prime * x_z) + (2 * p1 * x_z * y_z) + p2 * (r2 + 2 * x_z2)};
    T const y_star{(r_prime * y_z) + (2 * p2 * x_z * y_z) + p1 * (r2 + 2 * y_z2)};

    return {x_star, y_star};
}

template <typename T>
Eigen::Vector<T, 2> PinholeRadtan4Projection(Eigen::Array<T, 8, 1> const& intrinsics,
                                             Eigen::Array<T, 3, 1> const& point) {
    T const& x{point[0]};
    T const& y{point[1]};
    T const& z{point[2]};

    T const x_z{x / z};
    T const y_z{y / z};
    Eigen::Array<T, 2, 1> const projected_point{x_z, y_z};

    Eigen::Array<T, 4, 1> const distortion{intrinsics.bottomRows(4)};
    Eigen::Array<T, 2, 1> const projected_point_distorted{Radtan4Distortion(distortion, projected_point)};

    Eigen::Array<T, 4, 1> const pinhole_intrinsics{intrinsics.topRows(4)};
    Eigen::Array<T, 3, 1> const point_star{projected_point_distorted[0], projected_point_distorted[1l],
                                           1};  // z=1 here because we already "projected" at the start

    return PinholeProjection<T>(pinhole_intrinsics, point_star);
}

}  // namespace reprojection::projection_functions