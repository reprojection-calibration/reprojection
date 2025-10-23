#include "ceres_xxx.hpp"

#include <gtest/gtest.h>

using namespace reprojection;

// TODO(Jack): Test the nonlinear refinement with noisy data to make sure the optimization executes more than one step!

// We test that a point on the optical axis (0,0,z) projects to the center of the image (cx, cy) and has residual zero.
TEST(PnpNonlinearRefinement, TestPinholeCostFunctionResidual) {
    // NOTE(Jack): The reason that we have these ugly unfamiliar std::arrays and calls to .data(), but nowhere else, is
    // because in this test we are essentially manually simulating all the magic that Ceres will do behind the scenes
    // for us, managing the memory and passing arguments etc. during the optimization process. It is my hope and vision
    // that these raw pointers etc. can be limited to testing, and not actually filter into the rest of the code if
    // handled smartly (ex. using Eigen::Map and Eigen::Ref).
    std::array<double, 4> const pinhole_intrinsics{600, 600, 360, 240};
    Eigen::Vector2d const pixel{pinhole_intrinsics[2], pinhole_intrinsics[3]};
    Eigen::Vector3d const point{0, 0, 10};  // Point that will project to the center of the image
    optimization::PinholeCostFunction const cost_function{pixel, point};

    std::array<double, 6> const pose{0, 0, 0, 0, 0, 0};
    std::array<double, 2> residual{};
    cost_function.operator()<double>(pinhole_intrinsics.data(), pose.data(), residual.data());

    EXPECT_FLOAT_EQ(residual[0], 0.0);
    EXPECT_FLOAT_EQ(residual[1], 0.0);
}

// NOTE: We do not test cost_function->Evaluate() in the following test because
// allocating the memory of the input pointers takes some thought, but cost_function->Evaluate()
// should be tested when there is interest and time :)
TEST(PnpNonlinearRefinement, TestPinholeCostFunctionCreate) {
    Eigen::Vector2d const pixel{360, 240};
    Eigen::Vector3d const point{0, 0, 10};
    ceres::CostFunction const* const cost_function{optimization::PinholeCostFunction::Create(pixel, point)};

    EXPECT_EQ(std::size(cost_function->parameter_block_sizes()), 2);
    EXPECT_EQ(cost_function->parameter_block_sizes()[0], 4);  // pinhole intrinsics
    EXPECT_EQ(cost_function->parameter_block_sizes()[1], 6);  // camera pose
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

TEST(PnpNonlinearRefinement, TestTransformPointsTranslation) {
    Eigen::Vector<double, 6> const tf{0, 0, 0, 1, 2, 3};  // Translation only
    Eigen::Vector3d const point{5, 10, 15};

    Eigen::Vector<double, 3> const transformed_point{optimization::TransformPoint<double>(tf, point)};

    EXPECT_FLOAT_EQ(transformed_point[0], 6.0);
    EXPECT_FLOAT_EQ(transformed_point[1], 12.0);
    EXPECT_FLOAT_EQ(transformed_point[2], 18.0);
}

TEST(PnpNonlinearRefinement, TestTransformPointsRotation) {
    Eigen::Vector<double, 6> const tf{0, 0, M_PI_2, 0, 0, 0};  // Rotation only
    Eigen::Vector<double, 3> const point{5, 10, 15};

    Eigen::Vector<double, 3> const transformed_point{optimization::TransformPoint<double>(tf, point)};

    EXPECT_FLOAT_EQ(transformed_point[0], -10.0);
    EXPECT_FLOAT_EQ(transformed_point[1], 5.0);
    EXPECT_FLOAT_EQ(transformed_point[2], 15.0);
}
