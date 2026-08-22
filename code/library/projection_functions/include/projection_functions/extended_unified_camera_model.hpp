#pragma once

#include "projection_functions/double_sphere.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::projection_functions {

/**
 * \ingroup projection_classes
 * \brief
 */
struct ExtendedUnifiedCameraModel {
    static int constexpr Size{5};

    static Eigen::Array<double, Size, 1> Initialize(double const gamma, double const height, double const width) {
        return {gamma, 0.5 * width, 0.5 * height, 1, 0.5};
    }

    template <typename T>
    static std::optional<Array2<T>> Project(Eigen::Array<T, Size, 1> const& intrinsics, ImageBounds const& bounds,
                                            Array3<T> const& P_co) {
        T const xi{0};
        Eigen::Array<T, 6, 1> const spherical_projection_intrinsics(intrinsics(0), intrinsics(1), intrinsics(2), xi,
                                                                    intrinsics(4), intrinsics(3));

        return SphericalProjection<T>(spherical_projection_intrinsics, bounds, P_co);
    }

    static std::optional<Array3d> Unproject(Eigen::Array<double, Size, 1> const& intrinsics, ImageBounds const& bounds,
                                            Array2d const& pixel);
};

}  // namespace reprojection::projection_functions