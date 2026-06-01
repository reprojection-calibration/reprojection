#include "rigid_body_angular_velocity.hpp"

#include <gtest/gtest.h>

using namespace reprojection;
using namespace reprojection::optimization::cost_functions;

TEST(OptimizationCostFunctions, TestRigidBodyAngularVelocityZeroResidual) {
    Vector3d const omega_imu{Vector3d::Zero()};
    RigidBodyAngularVelocity const cost_function{omega_imu, 0, 1};

    Array3d const aa_imu_co{Array3d::Zero()};
    Array6d const control_point{Array6d::Zero()};

    Array3d residual{-1, -1, -1};
    bool const success{cost_function(aa_imu_co.data(), control_point.data(), control_point.data(), control_point.data(),
                                     control_point.data(), residual.data())};
    EXPECT_TRUE(success);
    EXPECT_FLOAT_EQ(residual[0], 0.0);
    EXPECT_FLOAT_EQ(residual[1], 0.0);
    EXPECT_FLOAT_EQ(residual[2], 0.0);
}

// One test with heuristic residual values with non-trivial rotation - canary in the coal mine test.
TEST(OptimizationCostFunctions, TestRigidBodyAngularVelocityHeuristic) {
    Vector3d const omega_imu{Vector3d::Zero()};
    RigidBodyAngularVelocity const cost_function{omega_imu, 0, 1};

    Array3d const aa_imu_co{Array3d::Zero()};
    Eigen::RowVectorXd const indices{Eigen::RowVectorXd::LinSpaced(4, 0, 4 - 1)};
    Eigen::MatrixXd control_points{indices.replicate(3, 1)};

    Array3d residual{-1, -1, -1};
    bool const success{cost_function(aa_imu_co.data(), control_points.col(0).data(), control_points.col(1).data(),
                                     control_points.col(2).data(), control_points.col(3).data(), residual.data())};
    EXPECT_TRUE(success);
    EXPECT_FLOAT_EQ(residual[0], -1e+09);
    EXPECT_FLOAT_EQ(residual[1], -1e+09);
    EXPECT_FLOAT_EQ(residual[2], -1e+09);
}

TEST(OptimizationCostFunctions, TestRigidBodyAngularVelocityCreate) {
    Vector3d const omega_imu{Vector3d::Zero()};

    ceres::CostFunction const* const cost_function{RigidBodyAngularVelocity::Create(omega_imu, 0, 1)};

    EXPECT_EQ(std::size(cost_function->parameter_block_sizes()), 5);
    EXPECT_EQ(cost_function->parameter_block_sizes()[0], 3);  // Axis angle rotation
    EXPECT_EQ(cost_function->parameter_block_sizes()[1], 3);  // so3 control point 1
    EXPECT_EQ(cost_function->parameter_block_sizes()[2], 3);  // so3 control point 2
    EXPECT_EQ(cost_function->parameter_block_sizes()[3], 3);  // so3 control point 3
    EXPECT_EQ(cost_function->parameter_block_sizes()[4], 3);  // so3 control point 4
    EXPECT_EQ(cost_function->num_residuals(), 3);
    delete cost_function;
}
