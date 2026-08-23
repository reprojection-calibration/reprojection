#pragma once

#include <ceres/ceres.h>

#include "projection_functions/pinhole.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::projection_functions {

template <typename T>
static bool ValidSphericalProjection(T const z, T const xi, T const alpha, T const d1) {
    T const w1{alpha <= 0.5 ? alpha / (1.0 - alpha) : (1.0 - alpha) / alpha};  // (Eqn. 45)
    T const w2{(w1 + xi) / ceres::sqrt(2.0 * w1 * xi + xi * xi + 1.0)};        // (Eqn. 44)

    // (Eqn. 43)
    if (z > -w2 * d1) {
        return true;
    } else {
        return false;
    }
}

// TODO(Jack): Do we have a better name for this? Spherical projection is a little misleading as sometimes we have two
// spheres and sometimes one sphere, and sometimes no spheres but actually an elipsoid.
template <typename T>
static std::optional<Array2<T>> SphericalProjection(Eigen::Array<T, 6, 1> const& intrinsics, ImageBounds const& bounds,
                                                    Array3<T> const& P_co) {
    T const& x{P_co[0]};
    T const& y{P_co[1]};
    T const& z{P_co[2]};

    T const& alpha{intrinsics[3]};
    T const& xi{intrinsics[4]};
    T const& beta{intrinsics[5]};

    T const xx{x * x};
    T const yy{y * y};
    T const r2{xx + yy};

    T const d1{ceres::sqrt((beta * r2) + (z * z))};
    if (not ValidSphericalProjection(z, xi, alpha, d1)) {
        return std::nullopt;
    }

    T const wz{(xi * d1) + z};  // wz==z for for ucm end eucm
    T const d2{ceres::sqrt((beta * r2) + (wz * wz))};
    T const z_star{(alpha * d2) + (1.0 - alpha) * wz};

    Array3<T> const P_star{x, y, z_star};

    return Pinhole::Project<T>(intrinsics.template head<3>(), bounds, P_star);
}

}  // namespace reprojection::projection_functions