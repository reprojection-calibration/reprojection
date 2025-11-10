#include "r3_spline_cost_function.hpp"

#include <gtest/gtest.h>

#include "types/eigen_types.hpp"

using namespace reprojection;

// COPY PASTED!!!!
double Squared(double const x) { return x * x; }

// NOTE(Jack): If you want to understand more about the ground truth values of position, velocity, and acceleration
// please see the "Spline_r3Spline, TestTemplatedEvaluateOnParabola" test :)
TEST(OptimizationR3SplineCostFunction, TestCreateR3SplineCostFunction) {
    double const u_i{0.5};
    std::uint64_t const delta_t_ns{1};

    // TODO(Jack): Is this column wise or row wise/does it matter?
    spline::Matrix3Kd const P{{-1, -0.5, 0.5, 1},
                              {Squared(-1), Squared(-0.5), Squared(0.5), Squared(1)},
                              {Squared(-1), Squared(-0.5), Squared(0.5), Squared(1)}};

    // Set up the required input data for the Evaluate() method (normally handled internally by ceres).
    double const* const P_ptr{P.data()};
    double const* const* const P_ptr_ptr{&P_ptr};
    Vector3d residual;

    // Position
    Array3d const position{0, 0.28125, 0.28125};
    ceres::CostFunction* cost_function{
        optimization::CreateR3SplineCostFunction(spline::DerivativeOrder::Null, position, u_i, delta_t_ns)};
    bool success{cost_function->Evaluate(P_ptr_ptr, residual.data(), nullptr)};  // Do not need jacobian, pass nullptr
    delete cost_function;
    EXPECT_TRUE(success);
    EXPECT_TRUE(residual.isZero());

    // Velocity
    Array3d const velocity{0.875, 0, 0};
    cost_function = optimization::CreateR3SplineCostFunction(spline::DerivativeOrder::First, velocity, u_i, delta_t_ns);
    success = cost_function->Evaluate(P_ptr_ptr, residual.data(), nullptr);
    delete cost_function;
    EXPECT_TRUE(success);
    EXPECT_TRUE(residual.isZero());

    // Acceleration
    Array3d const acceleration{0, 0.75, 0.75};
    cost_function =
        optimization::CreateR3SplineCostFunction(spline::DerivativeOrder::Second, acceleration, u_i, delta_t_ns);
    success = cost_function->Evaluate(P_ptr_ptr, residual.data(), nullptr);
    delete cost_function;
    EXPECT_TRUE(success);
    EXPECT_TRUE(residual.isZero());
}

TEST(OptimizationR3SplineCostFunction, TestR3SplineCostFunctionCreate_T) {
    Vector3d const r3{0, 0, 0};
    double const u_i{0.2};
    std::uint64_t const delta_t_ns{5};

    ceres::CostFunction const* const cost_function{
        optimization::R3SplineCostFunction_T<spline::DerivativeOrder::Null>::Create(r3, u_i, delta_t_ns)};

    EXPECT_EQ(std::size(cost_function->parameter_block_sizes()), 1);
    EXPECT_EQ(cost_function->parameter_block_sizes()[0], 12);
    EXPECT_EQ(cost_function->num_residuals(), 3);
    delete cost_function;
}
