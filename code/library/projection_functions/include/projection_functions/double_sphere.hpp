#pragma once

#include <ceres/ceres.h>

#include "projection_functions/pinhole.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::projection_functions {

/**
 * \ingroup projection_classes
 * \brief Implemented following "The Double Sphere Camera Model" (https://arxiv.org/pdf/1807.08957)
 */
struct DoubleSphere {
    static int constexpr Size{6};

    template <typename T>
    static std::optional<Array2<T>> Project(Eigen::Array<T, Size, 1> const& intrinsics, Array3<T> const& P_co) {
        T const& x{P_co[0]};
        T const& y{P_co[1]};
        T const& z{P_co[2]};

        T const xx{x * x};
        T const yy{y * y};
        T const r2{xx + yy};
        T const d1{ceres::sqrt(r2 + z * z)};

        T const& xi{intrinsics[4]};
        T const wz{xi * d1 + z};  // wz = "weighted z"
        T const d2{ceres::sqrt(r2 + wz * wz)};

        T const& alpha{intrinsics[5]};
        T const z_star{(alpha * d2) + (1.0 - alpha) * wz};
        Array3<T> const P_star{x, y, z_star};

        return Pinhole::Project<T>(intrinsics.topRows(4), P_star);
    }

    static Array3d Unproject(Eigen::Array<double, Size, 1> const& intrinsics, Array2d const& pixel);
};

}  // namespace reprojection::projection_functions