#pragma once

#include <ceres/autodiff_cost_function.h>

#include "projection_functions/projection_class_concept.hpp"
#include "spline/se3_spline.hpp"
#include "spline/types.hpp"
#include "types/calibration_types.hpp"
#include "types/eigen_types.hpp"
#include "types/enums.hpp"

#include "ceres_geometry.hpp"
#include "projection_cost_function.hpp"

namespace reprojection::optimization {

ceres::CostFunction* Create(CameraModel const projection_type, ImageBounds const& bounds, Vector2d const& pixel,
                            Vector3d const& point, double const u_i, uint64_t const delta_t_ns);

template <typename T_Model>
    requires projection_functions::ProjectionClass<T_Model>
class SplineProjectionCostFunction_T {
   public:
    SplineProjectionCostFunction_T(Vector2d const& pixel, Vector3d const& point, ImageBounds const& bounds,
                                   double const u_i, uint64_t const delta_t_ns)
        : pixel_{pixel}, point_{point}, bounds_{bounds}, u_i_{u_i}, delta_t_ns_{delta_t_ns} {}

    template <typename T>
    bool operator()(T const* const intrinsics_ptr, T const* const control_point_0_ptr,
                    T const* const control_point_1_ptr, T const* const control_point_2_ptr,
                    T const* const control_point_3_ptr, T* const residual) const {
        // Map control point pointers into a usable control point matrix block
        std::array<T const* const, spline::constants::order> ptrs{control_point_0_ptr, control_point_1_ptr,
                                                                  control_point_2_ptr, control_point_3_ptr};
        spline::Matrix2NK<T> control_points;
        for (int i{0}; i < spline::constants::order; ++i) {
            control_points.col(i) = Eigen::Map<Eigen::Vector<T, 6> const>(ptrs[i], 6, 1);
        }

        Array6<T> const pose{spline::Se3Spline::EvaluatePose<T>(control_points, u_i_, delta_t_ns_)};

        return ProjectionCostFunction_T<T_Model>(pixel_, point_, bounds_)
            .template operator()<T>(intrinsics_ptr, pose.data(), residual);
    }

    static ceres::CostFunction* Create(Vector2d const& pixel, Vector3d const& point, ImageBounds const& bounds,
                                       double const u_i, uint64_t const delta_t_ns) {
        return new ceres::AutoDiffCostFunction<SplineProjectionCostFunction_T, 2, T_Model::Size, 6, 6, 6, 6>(
            new SplineProjectionCostFunction_T(pixel, point, bounds, u_i, delta_t_ns));
    }

    Vector2d pixel_;
    Vector3d point_;
    ImageBounds bounds_;

    double u_i_;
    uint64_t delta_t_ns_;
};

}  // namespace  reprojection::optimization