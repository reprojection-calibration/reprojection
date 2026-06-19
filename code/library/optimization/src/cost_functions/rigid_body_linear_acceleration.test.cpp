#include "rigid_body_linear_acceleration.hpp"

#include <gtest/gtest.h>

using namespace reprojection;
using namespace reprojection::optimization::cost_functions;

TEST(OptimizationCostFunctions, TestRigidBodyAngularVelocityGravityResidual) {
    Vector3d const omega_imu{Vector3d::Zero()};
    RigidBodyLinearAcceleration const cost_function{omega_imu, 0, 1};

    Array6d const tf_imu_co{Array6d::Zero()};
    Array3d const gravity_w{0, 0, -9.81};
    Array6d const control_point{Array6d::Zero()};

    Array3d residual{-1, -1, -1};
    bool const success{cost_function(tf_imu_co.data(), gravity_w.data(), control_point.data(), control_point.data(),
                                     control_point.data(), control_point.data(), residual.data())};
    EXPECT_TRUE(success);
    EXPECT_FLOAT_EQ(residual[0], 0.0);
    EXPECT_FLOAT_EQ(residual[1], 0.0);
    EXPECT_FLOAT_EQ(residual[2], 9.81);
}

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
    EXPECT_EQ(cost_function->num_residuals(), 4);
    delete cost_function;
}
