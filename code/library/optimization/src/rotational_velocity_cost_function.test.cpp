#include "rotational_velocity_cost_function.hpp"

#include <gtest/gtest.h>

using namespace reprojection;

TEST(OptimizationProjectionCostFunction, TestRotationalVelocityCostFunction) {
    Array3d const aa_a_b{0, 0, 0};
    Array3d residual{-1, -1, -1};

    Vector3d const omega_a{Vector3d::Zero()};
    Vector3d const omega_b{Vector3d::Zero()};
    optimization::RotationalVelocityCostFunction const cost_function{omega_a, omega_b};
    bool const success{cost_function(aa_a_b.data(), residual.data())};
    EXPECT_TRUE(success);
    EXPECT_FLOAT_EQ(residual[0], 0.0);
    EXPECT_FLOAT_EQ(residual[1], 0.0);
    EXPECT_FLOAT_EQ(residual[2], 0.0);
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
