#pragma once

#include <Eigen/Core>

namespace reprojection::projection_functions {

template <typename T>
Eigen::Vector<T, 2> PinholeProjection(Eigen::Array<T, 4, 1> const& intrinsics, Eigen::Array<T, 3, 1> const& point) {
    T const& fx{intrinsics[0]};
    T const& fy{intrinsics[1]};
    T const& cx{intrinsics[2]};
    T const& cy{intrinsics[3]};

    T const& x{point[0]};
    T const& y{point[1]};
    T const& z{point[2]};

    T const u{(fx * x / z) + cx};
    T const v{(fy * y / z) + cy};

    return {u, v};
}

Eigen::MatrixX2d PinholeProjection(Eigen::Matrix3d const& K, Eigen::MatrixX3d points);

template <typename T>
Eigen::Vector<T, 3> PinholeUnrojection(Eigen::Array<T, 4, 1> const& intrinsics, Eigen::Array<T, 2, 1> const& pixel) {
    T const& fx{intrinsics[0]};
    T const& fy{intrinsics[1]};
    T const& cx{intrinsics[2]};
    T const& cy{intrinsics[3]};

    T const& u{pixel[0]};
    T const& v{pixel[1]};

    T const x{(u - cx) / fx};
    T const y{(v - cy) / fy};

    // NOTE(Jack): These rays do not have unit length, instead they are on the image plane at z=1.
    return {x, y, 1};
}

}  // namespace reprojection::projection_functions