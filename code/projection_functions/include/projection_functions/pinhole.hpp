#pragma once

#include <Eigen/Core>

namespace reprojection::projection_functions {

// TODO(Jack): What is the final and best type for camera going to be? Raw pointer smells to me, or is at least not 100%
// necessary considering how far along with ceres we are (not far).
template <typename T>
Eigen::Vector<T, 2> PinholeProjection(Eigen::Array<T, 4, 1> const& camera, Eigen::Array<T, 3, 1> const& point) {
    T const& fx{camera[0]};
    T const& fy{camera[1]};
    T const& cx{camera[2]};
    T const& cy{camera[3]};

    T const& x{point[0]};
    T const& y{point[1]};
    T const& z{point[2]};

    // TODO(Jack): Can/should we replace this with eigen matrix operations?
    T const u{(fx * x / z) + cx};
    T const v{(fy * y / z) + cy};

    return {u, v};
}

Eigen::MatrixX2d PinholeProjection(Eigen::Matrix3d const& K, Eigen::MatrixX3d points);

}  // namespace reprojection::projection_functions