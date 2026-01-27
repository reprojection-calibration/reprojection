#include "projection_cost_function.hpp"

#include <gtest/gtest.h>

#include "projection_functions/double_sphere.hpp"
#include "projection_functions/pinhole.hpp"
#include "projection_functions/pinhole_radtan4.hpp"
#include "projection_functions/unified_camera_model.hpp"
#include "testing_utilities/constants.hpp"
#include "types/algorithm_types.hpp"
#include "types/calibration_types.hpp"

using namespace reprojection;

// NOTE(Jack): Normally the cost functions are attached to a ceres "problem" object which takes ownership of them and is
// then responsible for deallocating them. This is actually controlled by the problem parameter option
// "Problem::Options::cost_function_ownership", for the full docs see
// (http://ceres-solver.org/nnls_modeling.html#_CPPv4N5ceres7Problem7Options23cost_function_ownershipE) and for a
// related stack exchange questions see
// (http://ceres-solver.org/nnls_modeling.html#_CPPv4N5ceres7Problem7Options23cost_function_ownershipE). In the testing
// here we do not attach the cost functions to a problem, therefore we have to call delete manually on each one we
// create! This violate RAII, and we should never let this pattern percolate into real library/application code!

TEST(OptimizationProjectionCostFunction, TestCreate) {
    Array2d const pixel{360, 240};
    Array3d const point{0, 0, 600};

    int const num_parameter_blocks{2};  // intrinsics and pose
    int const pose_size{6};             // se3 pose
    int const residual_size{2};         // pixel size: {u, v}

    ceres::CostFunction* cost_function{
        optimization::Create(CameraModel::DoubleSphere, testing_utilities::image_bounds, pixel, point)};
    EXPECT_EQ(std::size(cost_function->parameter_block_sizes()), num_parameter_blocks);
    EXPECT_EQ(cost_function->parameter_block_sizes()[0], projection_functions::DoubleSphere::Size);
    EXPECT_EQ(cost_function->parameter_block_sizes()[1], pose_size);
    EXPECT_EQ(cost_function->num_residuals(), residual_size);
    delete cost_function;

    cost_function = optimization::Create(CameraModel::Pinhole, testing_utilities::image_bounds, pixel, point);
    EXPECT_EQ(std::size(cost_function->parameter_block_sizes()), num_parameter_blocks);
    EXPECT_EQ(cost_function->parameter_block_sizes()[0], projection_functions::Pinhole::Size);
    EXPECT_EQ(cost_function->parameter_block_sizes()[1], pose_size);
    EXPECT_EQ(cost_function->num_residuals(), residual_size);
    delete cost_function;

    cost_function = optimization::Create(CameraModel::PinholeRadtan4, testing_utilities::image_bounds, pixel, point);
    EXPECT_EQ(std::size(cost_function->parameter_block_sizes()), num_parameter_blocks);
    EXPECT_EQ(cost_function->parameter_block_sizes()[0], projection_functions::PinholeRadtan4::Size);
    EXPECT_EQ(cost_function->parameter_block_sizes()[1], pose_size);
    EXPECT_EQ(cost_function->num_residuals(), residual_size);
    delete cost_function;

    cost_function =
        optimization::Create(CameraModel::UnifiedCameraModel, testing_utilities::image_bounds, pixel, point);
    EXPECT_EQ(std::size(cost_function->parameter_block_sizes()), num_parameter_blocks);
    EXPECT_EQ(cost_function->parameter_block_sizes()[0], projection_functions::UnifiedCameraModel::Size);
    EXPECT_EQ(cost_function->parameter_block_sizes()[1], pose_size);
    EXPECT_EQ(cost_function->num_residuals(), residual_size);
    delete cost_function;
}

// NOTE(Jack): The reason that we have these calls to .data(), but nowhere else, is because in this test we are
// essentially manually simulating all the magic that Ceres will do behind the scenes for us, managing the memory
// and passing arguments etc. during the optimization process. It is my hope and vision that these raw pointers etc.
// can be limited to testing, and not actually filter into the rest of the code if handled smartly (ex. using
// Eigen::Map and Eigen::Ref).
//
// We test that a point on the optical axis (0,0,z) projects to the center of the image (cx, cy) succesfully and has
// residual zero and that a point behind the camera returns false.
TEST(OptimizationProjectionCostFunction, TestProjectionCostFunction_T) {
    using PinholeCostFunction = optimization::ProjectionCostFunction_T<projection_functions::Pinhole>;
    Array2d const pixel{testing_utilities::pinhole_intrinsics[2], testing_utilities::pinhole_intrinsics[3]};
    Array6d const pose{0, 0, 0, 0, 0, 0};
    Array2d residual{-1, -1};

    // Point on front of the camera that will project to the center of the image.
    Array3d const point{0, 0, 10};
    PinholeCostFunction const cost_function{pixel, point, testing_utilities::image_bounds};
    bool success{cost_function(testing_utilities::pinhole_intrinsics.data(), pose.data(), residual.data())};
    EXPECT_TRUE(success);
    EXPECT_FLOAT_EQ(residual[0], 0.0);
    EXPECT_FLOAT_EQ(residual[1], 0.0);

    // A point behind the camera will return true but with a residual of 256 - see note in projection cost functions.
    Array3d const point_behind{0, 0, -10};
    PinholeCostFunction const cost_function_behind{pixel, point_behind, testing_utilities::image_bounds};
    success = cost_function_behind(testing_utilities::pinhole_intrinsics.data(), pose.data(), residual.data());
    EXPECT_TRUE(success);
    EXPECT_FLOAT_EQ(residual[0], 256);
    EXPECT_FLOAT_EQ(residual[1], 256);
}

// NOTE(Jack): We do not test cost_function->Evaluate() in the following test because allocating the memory of the input
// pointers takes some thought, but cost_function->Evaluate() should be tested when there is interest and time! Note
// that the logic required to allocate the required input parameters for the evaluate function is found in
// projection_functions:: PinholeRadtan4::JacobianUpdate().
TEST(OptimizationProjectionCostFunction, TestPinholeCreate_T) {
    Array2d const pixel{360, 240};
    Array3d const point{0, 0, 600};
    ceres::CostFunction const* const cost_function{
        optimization::ProjectionCostFunction_T<projection_functions::Pinhole>::Create(pixel, point,
                                                                                      testing_utilities::image_bounds)};

    EXPECT_EQ(std::size(cost_function->parameter_block_sizes()), 2);
    EXPECT_EQ(cost_function->parameter_block_sizes()[0], 4);  // pinhole intrinsics
    EXPECT_EQ(cost_function->parameter_block_sizes()[1], 6);  // camera pose
    EXPECT_EQ(cost_function->num_residuals(), 2);
    delete cost_function;
}
