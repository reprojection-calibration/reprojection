#include "spline_projection_cost_function.hpp"

#include <gtest/gtest.h>

#include "projection_functions/pinhole.hpp"
#include "testing_utilities/constants.hpp"

using namespace reprojection;

TEST(OptimizationSplineProjectionCostFunction, TestSplineProjectionCostFunction_T) {
    Array2d const pixel{testing_utilities::pinhole_intrinsics[2], testing_utilities::pinhole_intrinsics[3]};
    Array3d const point{0, 0, 10};
    optimization::SplineProjectionCostFunction_T<projection_functions::Pinhole> const cost_function{
        pixel, point, testing_utilities::image_bounds, 0, 1};

    // Test with four of the same identity control points. This should give us an identity SE3 pose.
    Array6d const control_point{Array6d::Zero()};
    Array2d residual{-1, -1};
    bool success{cost_function(testing_utilities::pinhole_intrinsics.data(), control_point.data(), control_point.data(),
                               control_point.data(), control_point.data(), residual.data())};
    EXPECT_TRUE(success);
    EXPECT_FLOAT_EQ(residual[0], 0.0);
    EXPECT_FLOAT_EQ(residual[1], 0.0);
}

TEST(OptimizationSplineProjectionCostFunction, TestSplineProjectionCostFunction_TCreate) {
    Array2d const pixel{360, 240};
    Array3d const point{0, 0, 600};
    ceres::CostFunction const* const cost_function{
        optimization::SplineProjectionCostFunction_T<projection_functions::Pinhole>::Create(
            pixel, point, testing_utilities::image_bounds, 0.0, 1)};

    EXPECT_EQ(std::size(cost_function->parameter_block_sizes()), 5);
    EXPECT_EQ(cost_function->parameter_block_sizes()[0], 4);  // pinhole intrinsics
    EXPECT_EQ(cost_function->parameter_block_sizes()[1], 6);  // control point 1
    EXPECT_EQ(cost_function->parameter_block_sizes()[2], 6);  // control point 2
    EXPECT_EQ(cost_function->parameter_block_sizes()[3], 6);  // control point 3
    EXPECT_EQ(cost_function->parameter_block_sizes()[4], 6);  // control point 4
    EXPECT_EQ(cost_function->num_residuals(), 2);
    delete cost_function;
}