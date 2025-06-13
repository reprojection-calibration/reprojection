#include <gtest/gtest.h>

#include "optimization_architecture/cost_functions.hpp"
#include "optimization_architecture/optimizer_handler.hpp"

using namespace reprojection_calibration::optimization_architecture;

TEST(TestOptimizerHandler, ParameterOverrides) {
    OneParameterCamera one_parameter{10.0};
    EXPECT_NEAR(one_parameter.GetParameterPtr()[0], 10.0, 1e-12);

    TwoParameterCamera two_parameter{{6.0, 4.0}};
    EXPECT_NEAR(two_parameter.GetParameterPtr()[0], 6.0, 1e-12);
    EXPECT_NEAR(two_parameter.GetParameterPtr()[1], 4.0, 1e-12);
}

TEST(TestOptimizerHandler, OptimizerHandlerCall) {
    double const data{10.0};

    OneParameterCamera one_parameter{0.0};
    OptimizerHandler(data, &one_parameter, &OneParameterCostFunction::Create);
    EXPECT_NEAR(data, one_parameter.GetParameterPtr()[0], 1e-6);

    // Given ground truth values, so the cost is zero and the optimizer only runs
    // once and terminates without modifying the parameters. The cost functions
    // are underconstrained so we do this to prevent hard coding heuristic ground
    // truth results into the test value.
    std::array<double, 2> const parameters{6.0, 4.0};
    TwoParameterCamera two_parameter{parameters};
    OptimizerHandler(data, &two_parameter, &TwoParameterCostFunction::Create);
    EXPECT_NEAR(parameters[0], two_parameter.GetParameterPtr()[0], 1e-6);
    EXPECT_NEAR(parameters[1], two_parameter.GetParameterPtr()[1], 1e-6);
}