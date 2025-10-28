#pragma once

#include "projection_functions/pinhole.hpp"
#include "types/eigen_types.hpp"

// Implemented following "The Double Sphere Camera Model" (https://arxiv.org/pdf/1807.08957)

namespace reprojection::projection_functions {

struct DoubleSphere {
    template <typename T>
    static Eigen::Vector<T, 2> Project(Eigen::Array<T, 6, 1> const& intrinsics, Eigen::Array<T, 3, 1> const& P_co) {
        T const& x{P_co[0]};
        T const& y{P_co[1]};
        T const& z{P_co[2]};

        T const xx{x * x};
        T const yy{y * y};
        T const r2{xx + yy};
        T const d1{std::sqrt(r2 + z * z)};

        T const& xi{intrinsics[4]};
        T const wz{xi * d1 + z};  // wz = "weighted z"
        T const d2{std::sqrt(r2 + wz * wz)};

        T const& alpha{intrinsics[5]};
        T const z_star{(alpha * d2) + (1.0 - alpha) * (xi * d1 + z)};
        Eigen::Vector<T, 3> const P_star{x, y, z_star};

        return Pinhole::Project<T>(intrinsics.topRows(4), P_star);
    }

    static Eigen::Vector<double, 3> Unproject(Array6d const& intrinsics, Array2d const& pixel);
};

}  // namespace reprojection::projection_functions