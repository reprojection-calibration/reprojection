#pragma once

#include "projection_functions/double_sphere.hpp"
#include "types/eigen_types.hpp"

// TODO(Jack): Double check that our assumption here about being able to represent the ucm with the ds is correct!
// TODO(Jack): Do the double sphere valid functions also cover the UCM? For example UCM sets ds alpha to zero, which
//  means the unprojection condition will never trigger. Is that ok?

namespace reprojection::projection_functions {

/**
 * \ingroup projection_classes
 * \brief
 */
struct UnifiedCameraModel {
    static int constexpr Size{4};

    static Eigen::Array<double, Size, 1> Initialize(double const gamma, double const height, double const width) {
        return {gamma, 0.5 * width, 0.5 * height, 0.5};
    }

    template <typename T>
    static std::optional<Array2<T>> Project(Eigen::Array<T, Size, 1> const& intrinsics, ImageBounds const& bounds,
                                            Array3<T> const& P_co) {
        T const xi{0};
        T const beta{1};
        Eigen::Array<T, 6, 1> const spherical_intrinsics(intrinsics(0), intrinsics(1), intrinsics(2), intrinsics(3), xi,
                                                         beta);

        return SphericalProjection<T>(spherical_intrinsics, bounds, P_co);
    }

    static std::optional<Array3d> Unproject(Eigen::Array<double, Size, 1> const& intrinsics, ImageBounds const& bounds,
                                            Array2d const& pixel);
};

}  // namespace reprojection::projection_functions