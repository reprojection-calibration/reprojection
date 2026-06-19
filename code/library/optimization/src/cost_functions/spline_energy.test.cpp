#include "spline_energy.hpp"

#include <gtest/gtest.h>

using namespace reprojection;
using namespace reprojection::optimization::cost_functions;

TEST(OptimizationCostFunctions, TestSplineEnergyCreate) {
    ceres::CostFunction const* const cost_function{SplineEnergy::Create(1)};

    EXPECT_EQ(std::size(cost_function->parameter_block_sizes()), 4);
    EXPECT_EQ(cost_function->parameter_block_sizes()[0], 6);  // control point 1
    EXPECT_EQ(cost_function->parameter_block_sizes()[1], 6);  // control point 2
    EXPECT_EQ(cost_function->parameter_block_sizes()[2], 6);  // control point 3
    EXPECT_EQ(cost_function->parameter_block_sizes()[3], 6);  // control point 4
    EXPECT_EQ(cost_function->num_residuals(), 24);
    delete cost_function;
}
