#include <gtest/gtest.h>

#include "spline_imu_cost_functions.hpp"

using namespace reprojection;

TEST(OptimizationSplineImuCostFunction, TestSplineImuCostFunctionCreate) {
    ImuData const imu_data{Vector3d::Zero(), Vector3d::Zero()};
    double const u_i{0};
    uint64_t const delta_t_ns{1};
    ceres::CostFunction const* const cost_function{
        optimization::SplineImuCostFunction::Create(imu_data, u_i, delta_t_ns)};

    int const num_parameter_blocks{6};  // cam-imu extrinsics, gravity, and four control points
    EXPECT_EQ(std::size(cost_function->parameter_block_sizes()), num_parameter_blocks);

    EXPECT_EQ(cost_function->parameter_block_sizes()[0], 6);  // cam-imu extrinsics
    EXPECT_EQ(cost_function->parameter_block_sizes()[1], 3);  // gravity
    EXPECT_EQ(cost_function->parameter_block_sizes()[2], 6);  // control point 1
    EXPECT_EQ(cost_function->parameter_block_sizes()[3], 6);  // control point 2
    EXPECT_EQ(cost_function->parameter_block_sizes()[4], 6);  // control point 3
    EXPECT_EQ(cost_function->parameter_block_sizes()[5], 6);  // control point 4
    EXPECT_EQ(cost_function->num_residuals(), 6);  // three for angular velocity and three for linear acceleration

    delete cost_function;
}

// Test with four of the same identity control points. This should give us an identity SE3 pose. Because the spline
// is stationary the specific values of u_i and delta_t_ns makes no difference.
TEST(OptimizationSplineImuCostFunction, TestSplineImuCostFunction) {
    ImuData const imu_data{Vector3d::Zero(), Vector3d::Zero()};
    double const u_i{0};
    uint64_t const delta_t_ns{1};
    optimization::SplineImuCostFunction const cost_function{imu_data, u_i, delta_t_ns};

    Array6d const tf_imu_co{Array6d::Zero()};
    Array3d const gravity_w{0, 0, 0};
    Array6d const control_point{Array6d::Zero()};
    Array6d residual{-1, -1, -1, -1, -1, -1};
    bool const success{cost_function(tf_imu_co.data(), gravity_w.data(), control_point.data(), control_point.data(),
                                     control_point.data(), control_point.data(), residual.data())};
    EXPECT_TRUE(success);
    EXPECT_FLOAT_EQ(residual[0], 0.0);
    EXPECT_FLOAT_EQ(residual[1], 0.0);
    EXPECT_FLOAT_EQ(residual[2], 0.0);
    EXPECT_FLOAT_EQ(residual[3], 0.0);
    EXPECT_FLOAT_EQ(residual[4], 0.0);
    EXPECT_FLOAT_EQ(residual[5], 0.0);
}