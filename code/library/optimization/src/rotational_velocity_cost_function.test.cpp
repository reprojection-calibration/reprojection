#include "rotational_velocity_cost_function.hpp"

#include <gtest/gtest.h>

using namespace reprojection;

// {aa_a_b, omega_a, omega_b}
std::vector<std::tuple<Array3d, Vector3d, Vector3d>> const test_cases{
    {{0, 0, 0}, Vector3d::Zero(), Vector3d::Zero()},  //
    // Identity rotation
    {{0, 0, 0}, Vector3d::UnitX(), Vector3d::UnitX()},
    // Rotate positive 90 degrees around the z-axis - vector goes from pointing positive x to positive y
    {{0, 0, M_PI / 2}, Vector3d::UnitY(), Vector3d::UnitX()},
    // Negative case gives opposite rotation direction
    {{0, 0, -M_PI / 2}, -Vector3d::UnitY(), Vector3d::UnitX()}};

TEST(OptimizationProjectionCostFunction, TestRotationalVelocityCostFunctionZeroResidual) {
    for (auto const& [aa_a_b, omega_a, omega_b] : test_cases) {
        Array3d residual{-1, -1, -1};
        optimization::RotationalVelocityCostFunction const cost_function{omega_a, omega_b};

        bool const success{cost_function(aa_a_b.data(), residual.data())};
        EXPECT_TRUE(success);
        EXPECT_NEAR(residual[0], 0.0, 1e-15);
        EXPECT_NEAR(residual[1], 0.0, 1e-15);
        EXPECT_NEAR(residual[2], 0.0, 1e-15);
    }
}

// One test with heuristic residual values with non-trivial rotation - canary in the coal mine test.
TEST(OptimizationProjectionCostFunction, TestRotationalVelocityCostFunction) {
    Vector3d const omega_a{0, 0, 0};
    Vector3d const omega_b{1, 2, 3};

    optimization::RotationalVelocityCostFunction const cost_function{omega_a, omega_b};

    Array3d const aa_a_b{0.1, 0.1, 0.1};
    Array3d residual{-1, -1, -1};
    bool const success{cost_function(aa_a_b.data(), residual.data())};
    EXPECT_TRUE(success);
    EXPECT_FLOAT_EQ(residual[0], -1.1144632869444262);
    EXPECT_FLOAT_EQ(residual[1], -1.8009985010709824);
    EXPECT_FLOAT_EQ(residual[2], -3.0845382119845914);
}

TEST(OptimizationRotationalVelocityCostFunction, TestRotationalVelocityCostFunctionCreate) {
    Vector3d const omega_a{Vector3d::Zero()};
    Vector3d const omega_b{Vector3d::Zero()};
    ceres::CostFunction const* const cost_function{
        optimization::RotationalVelocityCostFunction::Create(omega_a, omega_b)};

    EXPECT_EQ(std::size(cost_function->parameter_block_sizes()), 1);
    EXPECT_EQ(cost_function->parameter_block_sizes()[0], 3);  // Axis angle rotation
    EXPECT_EQ(cost_function->num_residuals(), 3);
    delete cost_function;
}
