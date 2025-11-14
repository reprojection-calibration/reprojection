#include "so3_spline_cost_function.hpp"

#include <ceres/ceres.h>

#include "spline/so3_spline.hpp"
#include "spline_cost_function.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::optimization {

ceres::CostFunction* CreateSo3SplineCostFunction(spline::DerivativeOrder const derivative, Vector3d const& r3,
                                                 double const u_i, std::uint64_t const delta_t_ns) {
    if (derivative == spline::DerivativeOrder::Null) {
        return SplineCostFunction_T<spline::So3Spline, spline::DerivativeOrder::Null>::Create(r3, u_i, delta_t_ns);
    } else if (derivative == spline::DerivativeOrder::First) {
        return SplineCostFunction_T<spline::So3Spline, spline::DerivativeOrder::First>::Create(r3, u_i, delta_t_ns);
    } else if (derivative == spline::DerivativeOrder::Second) {
        return SplineCostFunction_T<spline::So3Spline, spline::DerivativeOrder::Second>::Create(r3, u_i, delta_t_ns);
    } else {
        throw std::runtime_error(
            "Requested unknown derivative order from CreateR3SplineCostFunction()");  // LCOV_EXCL_LINE
    }
}

}  // namespace reprojection::optimization
