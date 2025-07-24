#include <gtest/gtest.h>

#include "reprojection/cost_functions.hpp"

using namespace reprojection_calibration::reprojection;

TEST(TestCostFunctions, PinholeCostFunctionResidual) {
    std::array<double, 2> const measured_pixel{250, 250};
    PinholeCostFunction const cost_function{measured_pixel[0], measured_pixel[1]};

    std::array<double, 4> const pinhole_intrinsics{100, 100, 250, 250};
    std::array<double, 6> const pose{0, 0, 0, 0, 0, 0};
    std::array<double, 3> const point{0, 0, 10};  // point that will project to exact center of the image
    std::array<double, 2> residual{};
    cost_function.operator()<double>(pinhole_intrinsics.data(), pose.data(), point.data(), residual.data());

    EXPECT_NEAR(residual[0], 0.0, 1e-6);
    EXPECT_NEAR(residual[1], 0.0, 1e-6);
}

// NOTE: We do not test cost_function->Evaluate() in the following test because
// allocating the memory of the input pointers takes some thought, but cost_function->Evaluate()
// should be tested when there is interest and time :)
TEST(TestCostFunctions, PinholeCostFunctionCreate) {
    std::array<double, 2> const measured_pixel{250, 250};
    ceres::CostFunction const* const cost_function{PinholeCostFunction::Create(measured_pixel[0], measured_pixel[1])};

    EXPECT_EQ(std::size(cost_function->parameter_block_sizes()), 3);
    EXPECT_EQ(cost_function->parameter_block_sizes()[0], 4);  // pinhole intrinsics
    EXPECT_EQ(cost_function->parameter_block_sizes()[1], 6);  // camera pose
    EXPECT_EQ(cost_function->parameter_block_sizes()[2], 3);  // 3D point
    EXPECT_EQ(cost_function->num_residuals(), 2);

    // WARN: The plain use of the ParameterCostFunction::Create() method does not
    // ensure the destruction of the resource allocated by the Create() method.
    // Therefore we need to call delete here, extra. Ceres normally handles this
    // when it uses create. But we do it here explicitly and without a class that
    // manages the memory allocation just cause this is a small part of a small
    // test. If we end up really ever using it outside of Ceres then we need to do
    // RAII.
    delete cost_function;
}

TEST(TestCostFunctions, UcmCostFunctionResidual) {
    std::array<double, 2> const measured_pixel{250, 250};
    UcmCostFunction const cost_function{measured_pixel[0], measured_pixel[1]};

    std::array<double, 5> const ucm_intrinsics{100, 100, 250, 250, 0.5};
    std::array<double, 6> const pose{0, 0, 0, 0, 0, 0};
    std::array<double, 3> const point{0, 0, 10};  // point that will project to exact center of the image
    std::array<double, 2> residual{};
    cost_function.operator()<double>(ucm_intrinsics.data(), pose.data(), point.data(), residual.data());

    EXPECT_NEAR(residual[0], 0.0, 1e-6);
    EXPECT_NEAR(residual[1], 0.0, 1e-6);
}

TEST(TestCostFunctions, UcmCostFunctionCreate) {
    std::array<double, 2> const measured_pixel{250, 250};
    ceres::CostFunction const* const cost_function{UcmCostFunction::Create(measured_pixel[0], measured_pixel[1])};

    EXPECT_EQ(std::size(cost_function->parameter_block_sizes()), 3);
    EXPECT_EQ(cost_function->parameter_block_sizes()[0], 5);  // ucm intrinsics
    EXPECT_EQ(cost_function->parameter_block_sizes()[1], 6);
    EXPECT_EQ(cost_function->parameter_block_sizes()[2], 3);
    EXPECT_EQ(cost_function->num_residuals(), 2);

    delete cost_function;
}