#pragma once

#include "projection_functions/double_sphere.hpp"
#include "types/eigen_types.hpp"

// TODO(Jack): Double check that our assumption here about being able to represent the ucm with the ds is correct!

namespace reprojection::projection_functions {

/**
 * \ingroup projection_classes
 * \brief Implemented as a degenerate case of the DoubleSphere model (xi=1, alpha=0).
 */
struct UnifiedCameraModel {
    static int constexpr Size{5};

    template <typename T>
    static std::optional<Array2<T>> Project(Eigen::Array<T, Size, 1> const& intrinsics, Array3<T> const& P_co) {
        T const alpha{0};  // Set alpha to zero - make ds equivalent to ucm by collapsing the second sphere in ds
        Eigen::Array<T, 6, 1> const ds_intrinsics(intrinsics(0), intrinsics(1), intrinsics(2), intrinsics(3),
                                                  intrinsics(4), alpha);

        return DoubleSphere::Project<T>(ds_intrinsics, P_co);
    }

    static Array3d Unproject(Eigen::Array<double, Size, 1> const& intrinsics, Array2d const& pixel);
};

}  // namespace reprojection::projection_functions