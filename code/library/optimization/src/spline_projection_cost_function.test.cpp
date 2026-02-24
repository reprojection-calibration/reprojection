#include "spline_projection_cost_function.hpp"

#include <gtest/gtest.h>

#include "projection_functions/pinhole.hpp"
#include "testing_utilities/constants.hpp"

using namespace reprojection;

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