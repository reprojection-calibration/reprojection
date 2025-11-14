#include "r3_spline_cost_function.hpp"

#include "spline_cost_function.hpp"

namespace reprojection::optimization {

// TODO(Jack): Confirm that in the calibration problem itself we will use all the following types of cost functions for
// r3, position, velocity and acceleration. I think we will use the "position" one for the IMU biases, but I am not so
// sure about the others. Or can we use all of these to construct the larger SE3 cost functions, or does it not make
// sense to compose them but to instead do them entirely by themselves.
ceres::CostFunction* CreateR3SplineCostFunction(spline::DerivativeOrder const derivative, Vector3d const& r3,
                                                double const u_i, std::uint64_t const delta_t_ns) {
    if (derivative == spline::DerivativeOrder::Null) {
        return SplineCostFunction_T<spline::R3Spline, spline::DerivativeOrder::Null>::Create(r3, u_i, delta_t_ns);
    } else if (derivative == spline::DerivativeOrder::First) {
        return SplineCostFunction_T<spline::R3Spline, spline::DerivativeOrder::First>::Create(r3, u_i, delta_t_ns);
    } else if (derivative == spline::DerivativeOrder::Second) {
        return SplineCostFunction_T<spline::R3Spline, spline::DerivativeOrder::Second>::Create(r3, u_i, delta_t_ns);
    } else {
        throw std::runtime_error(
            "Requested unknown derivative order from CreateR3SplineCostFunction()");  // LCOV_EXCL_LINE
    }
}

}  // namespace reprojection::optimization
