#include "so3_spline_cost_function.hpp"

#include <gtest/gtest.h>

#include "spline/types.hpp"
#include "types/eigen_types.hpp"

using namespace reprojection;

double Squared(double const x) { return x * x; }  // COPY PASTED!!!!

// Look in the so3 spline units test/r3 unit test to understand the testing values and philsophy.
TEST(OptimizationSo3SplineCostFunction, TestCreateSo3SplineCostFunction) {
    double const u_i{0.5};
    std::uint64_t const delta_t_ns{1};

    spline::Matrix3Kd const P{{-1, -0.5, 0.5, 1},
                              {Squared(-1), Squared(-0.5), Squared(0.5), Squared(1)},
                              {Squared(-1), Squared(-0.5), Squared(0.5), Squared(1)}};

    // Set up the required input data for the Evaluate() method (normally handled internally by ceres).
    double const* const P1_ptr{P.col(0).data()};
    double const* const P2_ptr{P.col(1).data()};
    double const* const P3_ptr{P.col(2).data()};
    double const* const P4_ptr{P.col(3).data()};
    double const* const P_ptr_ptr[4]{P1_ptr, P2_ptr, P3_ptr, P4_ptr};
    Vector3d residual;

    // Position
    Array3d const position{1.0319672855968482, -0.40184576778254827, -0.89024132986881044};
    ceres::CostFunction* cost_function{
        optimization::CreateSo3SplineCostFunction(spline::DerivativeOrder::Null, position, u_i, delta_t_ns)};
    bool success{cost_function->Evaluate(P_ptr_ptr, residual.data(), nullptr)};
    delete cost_function;
    EXPECT_TRUE(success);
    EXPECT_TRUE(residual.isZero());

    // Velocity
    Array3d const velocity{0.8563971186898035, -0.1204280865611993, 0.12722122556164611};
    cost_function =
        optimization::CreateSo3SplineCostFunction(spline::DerivativeOrder::First, velocity, u_i, delta_t_ns);
    success = cost_function->Evaluate(P_ptr_ptr, residual.data(), nullptr);
    delete cost_function;
    EXPECT_TRUE(success);
    EXPECT_TRUE(residual.isZero());

    // Acceleration
    Array3d const acceleration{0.0069974409407700944, 0.80095289350156396, 0.71108131312833733};
    cost_function =
        optimization::CreateSo3SplineCostFunction(spline::DerivativeOrder::Second, acceleration, u_i, delta_t_ns);
    success = cost_function->Evaluate(P_ptr_ptr, residual.data(), nullptr);
    delete cost_function;
    EXPECT_TRUE(success);
    EXPECT_TRUE(residual.isZero());
}

TEST(OptimizationSo3SplineCostFunction, TestSo3SplineAutodiffEquivalence) {
    Vector3d const r3{0, 0, 0};
    double const u_i{0.5};
    std::uint64_t const delta_t_ns{1};

    spline::Matrix3Kd const P{{-1, -0.5, 0.5, 1},
                              {Squared(-1), Squared(-0.5), Squared(0.5), Squared(1)},
                              {Squared(-1), Squared(-0.5), Squared(0.5), Squared(1)}};
    std::vector<double const*> parameter_blocks;
    parameter_blocks.push_back(P.col(0).data());
    parameter_blocks.push_back(P.col(1).data());
    parameter_blocks.push_back(P.col(2).data());
    parameter_blocks.push_back(P.col(3).data());

    ceres::CostFunction const* const position_cost_function{
        optimization::So3SplineCostFunction_T<spline::DerivativeOrder::Null>::Create(r3, u_i, delta_t_ns)};
    ceres::NumericDiffOptions const numeric_diff_options;
    ceres::GradientChecker const position_gradient_checker(position_cost_function, nullptr, numeric_diff_options);

    ceres::GradientChecker::ProbeResults results;
    bool const good_position_gradient{position_gradient_checker.Probe(parameter_blocks.data(), 1e-9, &results)};
    delete position_cost_function;
    EXPECT_TRUE(good_position_gradient) << results.error_log;

    ceres::CostFunction const* const velocity_cost_function{
        optimization::So3SplineCostFunction_T<spline::DerivativeOrder::First>::Create(r3, u_i, delta_t_ns)};
    ceres::GradientChecker const velocity_gradient_checker(velocity_cost_function, nullptr, numeric_diff_options);

    bool const good_velocity_gradient{velocity_gradient_checker.Probe(parameter_blocks.data(), 1e-9, &results)};
    delete velocity_cost_function;
    EXPECT_TRUE(good_velocity_gradient) << results.error_log;

    ceres::CostFunction const* const acceleration_cost_function{
        optimization::So3SplineCostFunction_T<spline::DerivativeOrder::Second>::Create(r3, u_i, delta_t_ns)};
    ceres::GradientChecker const acceleration_gradient_checker(acceleration_cost_function, nullptr, numeric_diff_options);

    bool const good_acceleration_gradient{acceleration_gradient_checker.Probe(parameter_blocks.data(), 1e-9, &results)};
    delete acceleration_cost_function;
    EXPECT_TRUE(good_acceleration_gradient) << results.error_log;
}

TEST(OptimizationSo3SplineCostFunction, TestSo3SplineCostFunctionCreate_T) {
    Vector3d const so3{0, 0, 0};
    double const u_i{0.5};
    std::uint64_t const delta_t_ns{1};

    ceres::CostFunction const* const cost_function{
        optimization::So3SplineCostFunction_T<spline::DerivativeOrder::Null>::Create(so3, u_i, delta_t_ns)};

    // Four r3 control point parameter blocks of size three and a r3 residual
    EXPECT_EQ(std::size(cost_function->parameter_block_sizes()), 4);
    EXPECT_EQ(cost_function->parameter_block_sizes()[0], 3);
    EXPECT_EQ(cost_function->parameter_block_sizes()[1], 3);
    EXPECT_EQ(cost_function->parameter_block_sizes()[2], 3);
    EXPECT_EQ(cost_function->parameter_block_sizes()[3], 3);
    EXPECT_EQ(cost_function->num_residuals(), 3);
    delete cost_function;
}
