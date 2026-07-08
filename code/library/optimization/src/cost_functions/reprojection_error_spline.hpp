#pragma once

#include <ceres/autodiff_cost_function.h>

#include "cost_functions/utils.hpp"
#include "projection_functions/projection_class_concept.hpp"
#include "spline/se3_spline.hpp"
#include "spline/types.hpp"
#include "types/calibration_types.hpp"
#include "types/eigen_types.hpp"
#include "types/enums.hpp"

#include "ceres_geometry.hpp"
#include "reprojection_error.hpp"

namespace reprojection::optimization::cost_functions {

ceres::CostFunction* Create(CameraModel const projection_type, ImageBounds const& bounds, Vector2d const& pixel,
                            Vector3d const& point, double const u_i, uint64_t const delta_t_ns);

template <typename T_Model>
    requires projection_functions::ProjectionClass<T_Model>
class ReprojectionErrorSpline_T {
   public:
    template <typename T>
    bool operator()(T const* const intrinsics_ptr, T const* const cp_0_ptr, T const* const cp_1_ptr,
                    T const* const cp_2_ptr, T const* const cp_3_ptr, T* const residual_ptr) const {
        auto const P{BuildP<T, 6>(cp_0_ptr, cp_1_ptr, cp_2_ptr, cp_3_ptr)};

        // Calculate the se3 pose and then return the standard reprojection error.
        Array6<T> const tf_w_co{spline::Se3Spline::EvaluatePose<T>(P, u_i_, delta_t_ns_)};
        Array6<T> const tf_co_w{geometry::InverseTransform(tf_w_co)};

        return ReprojectionError_T<T_Model>(pixel_, point_w_, bounds_)
            .template operator()<T>(intrinsics_ptr, tf_co_w.data(), residual_ptr);
    }

    static ceres::CostFunction* Create(Vector2d const& pixel, Vector3d const& point_w, ImageBounds const& bounds,
                                       double const u_i, uint64_t const delta_t_ns) {
        return new ceres::AutoDiffCostFunction<ReprojectionErrorSpline_T, 2, T_Model::Size, 6, 6, 6, 6>(
            new ReprojectionErrorSpline_T(pixel, point_w, bounds, u_i, delta_t_ns));
    }

    Vector2d pixel_;
    Vector3d point_w_;
    ImageBounds bounds_;

    double u_i_;
    uint64_t delta_t_ns_;
};

}  // namespace reprojection::optimization::cost_functions