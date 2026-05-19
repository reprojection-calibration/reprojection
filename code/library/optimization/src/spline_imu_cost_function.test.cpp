#include <gtest/gtest.h>

#include "spline_imu_cost_functions.hpp"

using namespace reprojection;

TEST(OptimizationSplineImuCostFunction, TestCreate) {
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
