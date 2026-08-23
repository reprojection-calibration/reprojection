#pragma once

#include <ceres/ceres.h>

#include "projection_functions/pinhole.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::projection_functions {

// NOTE(Jack): In "The Double Sphere Camera Model, Usenko Et al. 2018" they formulate the UCM, EUCM and double sphere
// camera models so clearly that we are able to implement all three of them using nearly the same exact code. THe basic
// idea is that the three camera models build off of each other. The UCM uses a sphere parameterized by alpha (or xi
// depending on notation, but we use the alpha parameterization), the EUCM turns that sphere into an ellipsoid, and the
// double sphere adds a second sphere around the UCMs first sphere. The following code implements this idea. Then
// important thing to understand, which I admit is a little confusing, is that you turn the generic "spherical
// projection" code into one of the above three camera models by selectively passing the parameters.
//
//  1) UCM - xi=0 and beta=1 - alpha is free
//  2) EUCM - xi=0 - alpha and beta are free
//  3) double sphere - beta=1 - alpha and xi are free
//
// If you look at the Project() function of each of these camera models you will see how their camera model specific
// intrinsics are fit into the spherical intrinsics to achieve the desired camera model.

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

std::optional<Array3d> SphericalUnprojection(Eigen::Array<double, 6, 1> const& intrinsics, ImageBounds const& bounds,
                                             Array2d const& pixel);

}  // namespace reprojection::projection_functions