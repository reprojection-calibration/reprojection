#include "r3_spline_cost_function.hpp"

namespace reprojection::optimization {

// NOTE(Jack): A general misfit idea is that we actually calculate the velocity and acceleration for the r3 spline
// analytically. Therefore for the position and velocity, we have the analytic derivatives! However, for the sake of
// consistency we are going to continue to use auto diff. If at a later date we find the performance is a problem, and
// we need analytic jacobians than the r3 spline optimization can be a nice place to start.
// TODO(Jack): Confirm that in the calibration problem itself we will use all the following types of cost functions for
// r3, position, velocity and acceleration. I think we will use the "position" one for the IMU biases, but I am not so
// sure about the others. Or can we use all of these to construct the larger SE3 cost functions, or does it not make
// sense to compose them but to instead do them entirely by themselves.
ceres::CostFunction* CreateR3SplineCostFunction(spline::DerivativeOrder const derivative, Vector3d const& r3,
                                                double const u_i, std::uint64_t const delta_t_ns) {
    if (derivative == spline::DerivativeOrder::Null) {
        return R3SplineCostFunction_T<spline::DerivativeOrder::Null>::Create(r3, u_i, delta_t_ns);
    } else if (derivative == spline::DerivativeOrder::First) {
        return R3SplineCostFunction_T<spline::DerivativeOrder::First>::Create(r3, u_i, delta_t_ns);
    } else if (derivative == spline::DerivativeOrder::Second) {
        return R3SplineCostFunction_T<spline::DerivativeOrder::Second>::Create(r3, u_i, delta_t_ns);
    } else {
        throw std::runtime_error("Requested unknown derivative order from CreateR3SplineCostFunction()");
    }
}

}  // namespace reprojection::optimization
