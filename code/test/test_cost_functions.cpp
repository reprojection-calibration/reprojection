#include <gtest/gtest.h>

#include "optimization_architecture/cost_functions.hpp"

using namespace reprojection_calibration::optimization_architecture;

TEST(TestCostFunctions, OneParameterCostFunctionResidual) {
    double const parameter{10.0};
    OneParameterCostFunction const cost_function{parameter};

    double residual{};
    cost_function.operator()<double>(&parameter, &residual);

    EXPECT_NEAR(residual, 0.0, 1e-6);
}

TEST(TestCostFunctions, TwoParameterCostFunctionResidual) {
    std::array<double, 2> const parameters{6.0, 4.0};
    TwoParameterCostFunction const cost_function{parameters[0] + parameters[1]};

    double residual{};
    cost_function.operator()<double>(parameters.data(), &residual);

    EXPECT_NEAR(residual, 0.0, 1e-6);
}

// NOTE: We do not test cost_function->Evaluate() in the following test because
// allocating the memory of the input pointers takes some thought, but the
// evaluate should be tested when there is interest and time :)
TEST(TestCostFunctions, OneParameterCostFunctionCreate) {
    double const parameter{10.0};
    ceres::CostFunction const* const cost_function{OneParameterCostFunction::Create(parameter)};

    EXPECT_EQ(std::size(cost_function->parameter_block_sizes()), 1);
    EXPECT_EQ(cost_function->parameter_block_sizes()[0], 1);
    EXPECT_EQ(cost_function->num_residuals(), 1);

    // WARN: The plain use of the ParameterCostFunction::Create() method does not
    // ensure the destruction of the resource allocated by the Create() method.
    // Therefore we need to call delete here, extra. Ceres normally handles this
    // when it uses create. But we do it here explicitly and without a class that
    // manages the memory allocation just cause this is a small part of a small
    // test. If we end up really ever using it outside of Ceres then we need to do
    // RAII.
    delete cost_function;
}

TEST(TestCostFunctions, TwoParameterCostFunctionCreate) {
    double const parameter{10.0};
    ceres::CostFunction const* const cost_function{TwoParameterCostFunction::Create(parameter)};

    EXPECT_EQ(std::size(cost_function->parameter_block_sizes()), 1);
    EXPECT_EQ(cost_function->parameter_block_sizes()[0], 2);
    EXPECT_EQ(cost_function->num_residuals(), 1);

    delete cost_function;
}
