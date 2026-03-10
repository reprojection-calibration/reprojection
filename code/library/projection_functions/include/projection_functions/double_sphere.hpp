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

    // TODO(Jack): The choice of initial xi and alpha is, at time of writing, totally arbitrary!
    // TODO(Jack): Add to projection concept!
    static Eigen::Array<double, Size, 1> Initialize(double const gamma, double const height, double const width) {
        return {0.5 * gamma, 0.5 * gamma, 0.5 * width, 0.5 * height, 0, 0.5};
    }

    template <typename T>
    static std::optional<Array2<T>> Project(Eigen::Array<T, Size, 1> const& intrinsics, ImageBounds const& bounds,
                                            Array3<T> const& P_co) {
        T const& x{P_co[0]};
        T const& y{P_co[1]};
        T const& z{P_co[2]};

        T const xx{x * x};
        T const yy{y * y};
        T const r2{xx + yy};
        T const d1{ceres::sqrt(r2 + z * z)};

        T const& xi{intrinsics[4]};
        T const& alpha{intrinsics[5]};

        if (not ValidProjection(z, xi, alpha, d1)) {
            return std::nullopt;
        }

        T const wz{xi * d1 + z};  // wz = "weighted z"
        T const d2{ceres::sqrt(r2 + wz * wz)};

        T const z_star{(alpha * d2) + (1.0 - alpha) * wz};
        Array3<T> const P_star{x, y, z_star};

        return Pinhole::Project<T>(intrinsics.template head<4>(), bounds, P_star);
    }

    // TODO(Jack): There is absolutely no reason why this needs to be templated! There is no need to get the ceres jet
    //  autdiff gradients in here. This function should just accept double type and the jets should be cast to double in
    //  the projection function below.,
    template <typename T>
    static bool ValidProjection(T const z, T const xi, T const alpha, T const d1) {
        T const w1{alpha <= 0.5 ? alpha / (1.0 - alpha) : (1.0 - alpha) / alpha};  // (Eqn. 45)
        T const w2{(w1 + xi) / ceres::sqrt(2.0 * w1 * xi + xi * xi + 1.0)};          // (Eqn. 44)

        // (Eqn. 43)
        if (z > -w2 * d1) {
            return true;
        } else {
            return false;
        }
    }

    static std::optional<Array3d> Unproject(Eigen::Array<double, Size, 1> const& intrinsics, ImageBounds const& bounds,
                                            Array2d const& pixel);
};

}  // namespace reprojection::projection_functions