#include <ceres/ceres.h>

#include "spline/so3_spline.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::optimization {

// Exact same as r3 case!
ceres::CostFunction* CreateSo3SplineCostFunction(spline::DerivativeOrder const derivative, Vector3d const& r3,
                                                 double const u_i, std::uint64_t const delta_t_ns);

// COPY AND PASTED FROM EXACT SAME AS FOR R3 case
template <spline::DerivativeOrder D>
class So3SplineCostFunction_T {
   public:
    So3SplineCostFunction_T(Vector3d const& r3, double const u_i, std::uint64_t const delta_t_ns)
        : r3_{r3}, u_i_{u_i}, delta_t_ns_{delta_t_ns} {}

    template <typename T>
    bool operator()(T const* const control_point_0_ptr, T const* const control_point_1_ptr,
                    T const* const control_point_2_ptr, T const* const control_point_3_ptr, T* const residual) const {
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p0(control_point_0_ptr);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p1(control_point_1_ptr);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p2(control_point_2_ptr);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p3(control_point_3_ptr);

        spline::Matrix3K<T> control_points;
        control_points << p0, p1, p2, p3;

        Eigen::Vector<T, 3> const r3{spline::So3SplineEvaluation::Evaluate<T, D>(control_points, u_i_, delta_t_ns_)};

        residual[0] = T(r3_[0]) - r3[0];
        residual[1] = T(r3_[1]) - r3[1];
        residual[2] = T(r3_[2]) - r3[2];

        return true;
    }

    static ceres::CostFunction* Create(Vector3d const& r3, double const u_i, std::uint64_t const delta_t_ns) {
        return new ceres::AutoDiffCostFunction<So3SplineCostFunction_T, 3, 3, 3, 3, 3>(
            new So3SplineCostFunction_T(r3, u_i, delta_t_ns));
    }

    Vector3d r3_;
    double u_i_;
    std::uint64_t delta_t_ns_;
};

}  // namespace reprojection::optimization
