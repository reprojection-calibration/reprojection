#include "rigid_body_linear_acceleration.hpp"

#include <gtest/gtest.h>

using namespace reprojection;
using namespace reprojection::optimization::cost_functions;

TEST(OptimizationCostFunctions, TestRigidBodyLinearAccelerationCreate) {
    Vector3d const acc_imu{Vector3d::Zero()};

    ceres::CostFunction const* const cost_function{RigidBodyLinearAcceleration::Create(acc_imu, 0, 1)};

    EXPECT_EQ(std::size(cost_function->parameter_block_sizes()), 6);
    EXPECT_EQ(cost_function->parameter_block_sizes()[0], 6);  // tf_co_imu
    EXPECT_EQ(cost_function->parameter_block_sizes()[1], 3);  // gravity
    EXPECT_EQ(cost_function->parameter_block_sizes()[2], 6);  // control point 1
    EXPECT_EQ(cost_function->parameter_block_sizes()[3], 6);  // control point 2
    EXPECT_EQ(cost_function->parameter_block_sizes()[4], 6);  // control point 3
    EXPECT_EQ(cost_function->parameter_block_sizes()[5], 6);  // control point 4
    EXPECT_EQ(cost_function->num_residuals(), 3);
    delete cost_function;
}
