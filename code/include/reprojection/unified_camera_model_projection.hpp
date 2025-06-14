#pragma once

#include <ceres/ceres.h>

#include "pinhole_projection.hpp"

namespace reprojection_calibration::reprojection {

template <typename T>
std::tuple<T, T> UcmProjection(T const* const camera, T const* const point) {
    T const& alpha{camera[4]};

    T const& x{point[0]};
    T const& y{point[1]};
    T const& z{point[2]};

    T const d{ceres::sqrt(x * x + y * y + z * z)};
    T const scaled_z{alpha * d + (1.0 - alpha) * z};

    std::array<T, 3> const scaled_point{x, y, scaled_z};

    return PinholeProjection(camera, scaled_point.data());
}

}  // namespace reprojection_calibration::reprojection