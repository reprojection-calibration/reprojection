#include "spline_energy.hpp"

#include <gtest/gtest.h>

#include "types/ceres_types.hpp"

using namespace reprojection;
using namespace reprojection::optimization::cost_functions;

TEST(OptimizationCostFunctions, TestSplineEnergyOptimization) {
    CeresState ceres_state{ceres::TAKE_OWNERSHIP, ceres::DENSE_SCHUR};
    ceres::Problem problem{ceres_state.problem_options};

    ceres::CostFunction* const cost_function{SplineEnergy::Create(1)};

    spline::Matrix2NK<double> control_points{spline::Matrix2NK<double>::Zero()};
    control_points.col(1).array() += 1;

    problem.AddResidualBlock(cost_function, nullptr,
                             control_points.col(0).data(),  //
                             control_points.col(1).data(), control_points.col(2).data(), control_points.col(3).data());

    // NOTE(Jack): Setting the first and last point constant here essentially forces the optimizer to optimize the
    // control points all back to zero. If we did not do this then it would optimize all the control points to be an
    // arbitrary collinear line. If we only set the first control point constant then it would be a collinear line that
    // intercepts the first control point. Setting the first and last control point fixed forces the control points all
    // to be optimized to zero which makes the test condition easier to assert.
    problem.SetParameterBlockConstant(control_points.col(0).data());
    problem.SetParameterBlockConstant(control_points.col(3).data());

    ceres::Solve(ceres_state.solver_options, &problem, &ceres_state.solver_summary);

    EXPECT_TRUE(control_points.isZero());
    EXPECT_NEAR(ceres_state.solver_summary.final_cost, 0, 1e-12);
}

TEST(OptimizationCostFunctions, TestSplineEnergyResidual) {
    spline::CoefficientBlock const omega{spline::BuildOmega(1, 1)};
    SplineEnergy const cost_function{omega};

    Array6d const control_point{Array6d::Zero()};

    Eigen::Array<double, 24, 1> residual{-Eigen::Array<double, 24, 1>::Ones()};
    bool const success{cost_function(control_point.data(), control_point.data(), control_point.data(),
                                     control_point.data(), residual.data())};

    EXPECT_TRUE(success);
    EXPECT_TRUE(residual.isApproxToConstant(0));
}

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
