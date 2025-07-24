#pragma once

#include <ceres/ceres.h>

#include "reprojection/geometric_transforms.hpp"
#include "reprojection/pinhole_projection.hpp"
#include "reprojection/unified_camera_model_projection.hpp"

namespace reprojection_calibration::reprojection {

// QUESTION(Jack): Many camera models are related to the pinhole camera model. However, when and where each camera model
// applies the projection is still unclear. If there is some consistency and the projection is applied at the same point
// of the process all the time it would be interesting to be able to reuse the entire pinhole cost function. This does
// would require us to move the Create functions out of the struct, but that is ok. It all depends if it really would
// bring us anything :) But there is copy and pasted code here, so we might be missing something.

// TODO(Jack): These are not plain old data types! These should be classes not structs.
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

struct UcmCostFunction {
    explicit UcmCostFunction(double const u, double const v) : u_{u}, v_{v} {}

    // WARN(Jack): I am pretty sure it is misleading to call these intrinsics "pinhole_intrinsics" because they are
    // actually Ucm intrinsics! The first four values are pinhole but the ones after that are the Ucm values.
    template <typename T>
    bool operator()(T const* const pinhole_intrinsics, T const* const pose, T const* const point,
                    T* const residual) const {
        std::array<T, 3> const point_co{TransformPoint(pose, point)};

        auto const [u, v]{UcmProjection(pinhole_intrinsics, point_co.data())};

        residual[0] = T(u_) - u;
        residual[1] = T(v_) - v;

        return true;
    }

    static ceres::CostFunction* Create(double const u, double const v) {
        return new ceres::AutoDiffCostFunction<UcmCostFunction, 2, 5, 6, 3>(new UcmCostFunction(u, v));
    }

    double u_;
    double v_;
};

}  // namespace reprojection_calibration::reprojection