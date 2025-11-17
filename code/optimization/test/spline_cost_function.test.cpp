#include "optimization/spline_cost_function.hpp"

#include <gtest/gtest.h>

#include "spline/r3_spline.hpp"
#include "spline/so3_spline.hpp"
#include "types/eigen_types.hpp"

using namespace reprojection;

// NOTE(Jack): If you want to see where the control point values and ground truth evaluation values below come from
// please look at the testing of the templated Evaluate() functions in the spline package.
class OptimizationSplineCostFunctionFixture : public ::testing::Test {
   protected:
    void SetUp() override {
        // This P_ matrix can be used by the templated static evaluation functions directly and serves as the storage
        // container for the data referenced to by the ceres friendly parameter blocks below.
        P_ = spline::Matrix3Kd{{-1, -0.5, 0.5, 1},
                               {Squared(-1), Squared(-0.5), Squared(0.5), Squared(1)},
                               {Squared(-1), Squared(-0.5), Squared(0.5), Squared(1)}};

        // The pointer indexed parameter blocks is the format needed by ceres.
        parameter_blocks_.push_back(P_.col(0).data());
        parameter_blocks_.push_back(P_.col(1).data());
        parameter_blocks_.push_back(P_.col(2).data());
        parameter_blocks_.push_back(P_.col(3).data());
    }

    double u_i_{0.5};
    std::uint64_t delta_t_ns_{1};
    spline::Matrix3Kd P_;
    std::vector<double const*> parameter_blocks_;

   private:
    double static Squared(double const x) { return x * x; }
};

void CheckSplineResidual(std::vector<double const*> const& parameter_blocks, ceres::CostFunction* cost_function) {
    Vector3d residual;

    bool success{cost_function->Evaluate(parameter_blocks.data(), residual.data(), nullptr)};
    delete cost_function;

    EXPECT_TRUE(success);
    EXPECT_TRUE(residual.isZero());
}

TEST_F(OptimizationSplineCostFunctionFixture, TestCreateSplineCostFunction_R3) {
    // Position
    Array3d const position{0, 0.28125, 0.28125};
    ceres::CostFunction* cost_function{optimization::CreateSplineCostFunction_T<spline::R3Spline>(
        spline::DerivativeOrder::Null, position, u_i_, delta_t_ns_)};
    CheckSplineResidual(parameter_blocks_, cost_function);

    // Velocity
    Array3d const velocity{0.875, 0, 0};
    cost_function = optimization::CreateSplineCostFunction_T<spline::R3Spline>(spline::DerivativeOrder::First, velocity,
                                                                               u_i_, delta_t_ns_);
    CheckSplineResidual(parameter_blocks_, cost_function);

    // Acceleration
    Array3d const acceleration{0, 0.75, 0.75};
    cost_function = optimization::CreateSplineCostFunction_T<spline::R3Spline>(spline::DerivativeOrder::Second,
                                                                               acceleration, u_i_, delta_t_ns_);
    CheckSplineResidual(parameter_blocks_, cost_function);
}

TEST_F(OptimizationSplineCostFunctionFixture, TestCreateSplineCostFunction_so3) {
    Vector3d residual;

    // Position
    Array3d const position{0.1460482362445171, 0.3755842237411095, 0.39702710822143839};
    ceres::CostFunction* cost_function{optimization::CreateSplineCostFunction_T<spline::So3Spline>(
        spline::DerivativeOrder::Null, position, u_i_, delta_t_ns_)};
    CheckSplineResidual(parameter_blocks_, cost_function);

    // Velocity
    Array3d const velocity{0.8563971186898035, -0.1204280865611993, 0.12722122556164611};
    cost_function = optimization::CreateSplineCostFunction_T<spline::So3Spline>(spline::DerivativeOrder::First,
                                                                                velocity, u_i_, delta_t_ns_);
    CheckSplineResidual(parameter_blocks_, cost_function);

    // Acceleration
    Array3d const acceleration{0.0069974409407700944, 0.80095289350156396, 0.71108131312833733};
    cost_function = optimization::CreateSplineCostFunction_T<spline::So3Spline>(spline::DerivativeOrder::Second,
                                                                                acceleration, u_i_, delta_t_ns_);
    CheckSplineResidual(parameter_blocks_, cost_function);
}

// WARN DELETES THE COST FUNCTION! USE SMART POINTER INSTEAD
void CheckSplineGradient(std::vector<double const*> const& parameter_blocks, ceres::CostFunction* cost_function) {
    ceres::NumericDiffOptions const numeric_diff_options;
    ceres::GradientChecker::ProbeResults results;

    ceres::GradientChecker const position_gradient_checker(cost_function, nullptr, numeric_diff_options);
    bool const good_position_gradient{position_gradient_checker.Probe(parameter_blocks.data(), 1e-9, &results)};
    delete cost_function;

    EXPECT_TRUE(good_position_gradient) << results.error_log;
}

// TODO(Jack): We violate RAII here with the cost functions, should we use unique pointers for the cost functions?
TEST_F(OptimizationSplineCostFunctionFixture, TestSplineGradients_R3) {
    // The gradient checker uses a relative error and cannot handle nearly exact zeros well, so we add 1e-6 to the first
    // position to avoid this problem for the r3 case where the x-values evaluate to zero given the control points.
    Vector3d const r3{1e-6, 0, 0};

    ceres::CostFunction* position{optimization::CreateSplineCostFunction_T<spline::R3Spline>(
        spline::DerivativeOrder::Null, r3, u_i_, delta_t_ns_)};
    CheckSplineGradient(parameter_blocks_, position);

    ceres::CostFunction* velocity{optimization::CreateSplineCostFunction_T<spline::R3Spline>(
        spline::DerivativeOrder::First, r3, u_i_, delta_t_ns_)};
    CheckSplineGradient(parameter_blocks_, velocity);

    ceres::CostFunction* acceleration{optimization::CreateSplineCostFunction_T<spline::R3Spline>(
        spline::DerivativeOrder::Second, r3, u_i_, delta_t_ns_)};
    CheckSplineGradient(parameter_blocks_, acceleration);
}

TEST_F(OptimizationSplineCostFunctionFixture, TestSplineGradients_so3) {
    Vector3d const so3{0, 0, 0};

    ceres::CostFunction* position{optimization::CreateSplineCostFunction_T<spline::So3Spline>(
        spline::DerivativeOrder::Null, so3, u_i_, delta_t_ns_)};
    CheckSplineGradient(parameter_blocks_, position);

    ceres::CostFunction* velocity{optimization::CreateSplineCostFunction_T<spline::So3Spline>(
        spline::DerivativeOrder::First, so3, u_i_, delta_t_ns_)};
    CheckSplineGradient(parameter_blocks_, velocity);

    ceres::CostFunction* acceleration{optimization::CreateSplineCostFunction_T<spline::So3Spline>(
        spline::DerivativeOrder::Second, so3, u_i_, delta_t_ns_)};
    CheckSplineGradient(parameter_blocks_, acceleration);
}

TEST(OptimizationSplineCostFunction, TestR3SplineCostFunctionCreate_T) {
    Vector3d const r3{0, 0, 0};
    double const u_i{0.5};
    std::uint64_t const delta_t_ns{1};

    ceres::CostFunction const* const cost_function{
        optimization::SplineCostFunction_T<spline::R3Spline, spline::DerivativeOrder::Null>::Create(r3, u_i,
                                                                                                    delta_t_ns)};

    EXPECT_EQ(std::size(cost_function->parameter_block_sizes()), 4);  // Four control points...
    EXPECT_EQ(cost_function->parameter_block_sizes()[0], 3);          // ... all of size three.
    EXPECT_EQ(cost_function->parameter_block_sizes()[1], 3);
    EXPECT_EQ(cost_function->parameter_block_sizes()[2], 3);
    EXPECT_EQ(cost_function->parameter_block_sizes()[3], 3);
    EXPECT_EQ(cost_function->num_residuals(), 3);  // One residual of size three.
    delete cost_function;
}

TEST(OptimizationSplineCostFunction, TestSo3SplineCostFunctionCreate_T) {
    Vector3d const so3{0, 0, 0};
    double const u_i{0.5};
    std::uint64_t const delta_t_ns{1};

    ceres::CostFunction const* const cost_function{
        optimization::SplineCostFunction_T<spline::So3Spline, spline::DerivativeOrder::Null>::Create(so3, u_i,
                                                                                                     delta_t_ns)};

    // See test above for explanation of the dimensions.
    EXPECT_EQ(std::size(cost_function->parameter_block_sizes()), 4);
    EXPECT_EQ(cost_function->parameter_block_sizes()[0], 3);
    EXPECT_EQ(cost_function->parameter_block_sizes()[1], 3);
    EXPECT_EQ(cost_function->parameter_block_sizes()[2], 3);
    EXPECT_EQ(cost_function->parameter_block_sizes()[3], 3);
    EXPECT_EQ(cost_function->num_residuals(), 3);
    delete cost_function;
}