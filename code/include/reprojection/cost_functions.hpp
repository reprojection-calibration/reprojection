#pragma once

#include <ceres/ceres.h>

#include "reprojection/geometric_transforms.hpp"
#include "reprojection/pinhole_projection.hpp"

namespace reprojection_calibration::reprojection {

struct PinholeCostFunction {
    explicit PinholeCostFunction(double const u, double const v) : u_{u}, v_{v} {}

    template <typename T>
    bool operator()(T const* const pinhole_intrinsics, T const* const pose, T const* const point,
                    T* const residual) const {
        std::array<T, 3> const point_co{TransformPoint(pose, point)};

        auto const [u, v]{PinholeProjection(pinhole_intrinsics, point_co.data())};

        residual[0] = T(u_) - u;
        residual[1] = T(v_) - v;

        return true;
    }

    static ceres::CostFunction* Create(double const u, double const v) {
        return new ceres::AutoDiffCostFunction<PinholeCostFunction, 2, 4, 6, 3>(new PinholeCostFunction(u, v));
    }

    double u_;
    double v_;
};

}  // namespace reprojection_calibration::reprojection