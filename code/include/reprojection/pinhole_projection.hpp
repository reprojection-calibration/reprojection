#pragma once

namespace reprojection_calibration::reprojection {

template <typename T>
std::tuple<T, T> PinholeProjection(T const* const camera, T const* const point) {
    T const& fx{camera[0]};
    T const& fy{camera[1]};
    T const& cx{camera[2]};
    T const& cy{camera[3]};

    T const& x{point[0]};
    T const& y{point[1]};
    T const& z{point[2]};

    T const u{(fx * x / z) + cx};
    T const v{(fy * y / z) + cy};

    return {u, v};
}

}  // namespace reprojection_calibration::reprojection