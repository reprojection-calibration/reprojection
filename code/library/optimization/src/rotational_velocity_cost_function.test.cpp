#include "rotational_velocity_cost_function.hpp"

#include <gtest/gtest.h>

using namespace reprojection;

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
