#include "projection_cost_function.hpp"

#include <gtest/gtest.h>

#include "projection_functions/double_sphere.hpp"
#include "projection_functions/pinhole.hpp"
#include "projection_functions/pinhole_radtan4.hpp"
#include "projection_functions/unified_camera_model.hpp"

using namespace reprojection;

// WARN(Jack): Is it possible that we are leaking memory with un-deallocated cost_function(s)?
TEST(OptimizationProjectionCostFunction, TestCreate) {
    Array2d const pixel{360, 240};
    Array3d const point{0, 0, 600};

    int const num_parameter_blocks{2};  // intrinsics and pose
    int const pose_size{6};             // se3
    int const residual_size{2};         // pixel_size

    ceres::CostFunction* cost_function{optimization::Create(optimization::CameraModel::DoubleSphere, pixel, point)};
    EXPECT_EQ(std::size(cost_function->parameter_block_sizes()), num_parameter_blocks);
    EXPECT_EQ(cost_function->parameter_block_sizes()[0], projection_functions::DoubleSphere::Size);
    EXPECT_EQ(cost_function->parameter_block_sizes()[1], pose_size);
    EXPECT_EQ(cost_function->num_residuals(), residual_size);

    cost_function = optimization::Create(optimization::CameraModel::Pinhole, pixel, point);
    EXPECT_EQ(std::size(cost_function->parameter_block_sizes()), num_parameter_blocks);
    EXPECT_EQ(cost_function->parameter_block_sizes()[0], projection_functions::Pinhole::Size);
    EXPECT_EQ(cost_function->parameter_block_sizes()[1], pose_size);
    EXPECT_EQ(cost_function->num_residuals(), residual_size);

    cost_function = optimization::Create(optimization::CameraModel::PinholeRadtan4, pixel, point);
    EXPECT_EQ(std::size(cost_function->parameter_block_sizes()), num_parameter_blocks);
    EXPECT_EQ(cost_function->parameter_block_sizes()[0], projection_functions::PinholeRadtan4::Size);
    EXPECT_EQ(cost_function->parameter_block_sizes()[1], pose_size);
    EXPECT_EQ(cost_function->num_residuals(), residual_size);

    cost_function = optimization::Create(optimization::CameraModel::UnifiedCameraModel, pixel, point);
    EXPECT_EQ(std::size(cost_function->parameter_block_sizes()), num_parameter_blocks);
    EXPECT_EQ(cost_function->parameter_block_sizes()[0], projection_functions::UnifiedCameraModel::Size);
    EXPECT_EQ(cost_function->parameter_block_sizes()[1], pose_size);
    EXPECT_EQ(cost_function->num_residuals(), residual_size);
}

// We test that a point on the optical axis (0,0,z) projects to the center of the image (cx, cy) and has residual zero.
TEST(OptimizationProjectionCostFunction, TestProjectionCostFunction_T) {
    // NOTE(Jack): The reason that we have these calls to .data(), but nowhere else, is because in this test we are
    // essentially manually simulating all the magic that Ceres will do behind the scenes for us, managing the memory
    // and passing arguments etc. during the optimization process. It is my hope and vision that these raw pointers etc.
    // can be limited to testing, and not actually filter into the rest of the code if handled smartly (ex. using
    // Eigen::Map and Eigen::Ref).
    Array4d const pinhole_intrinsics{600, 600, 360, 240};
    Array2d const pixel{pinhole_intrinsics[2], pinhole_intrinsics[3]};
    Array3d const point{0, 0, 10};  // Point that will project to the center of the image

    optimization::ProjectionCostFunction_T<projection_functions::Pinhole> const cost_function{pixel, point};

    Array6d const pose{0, 0, 0, 0, 0, 0};
    Array2d residual{-1, -1};
    cost_function.operator()<double>(pinhole_intrinsics.data(), pose.data(), residual.data());

    EXPECT_FLOAT_EQ(residual[0], 0.0);
    EXPECT_FLOAT_EQ(residual[1], 0.0);
}

// NOTE: We do not test cost_function->Evaluate() in the following test because
// allocating the memory of the input pointers takes some thought, but cost_function->Evaluate()
// should be tested when there is interest and time :)
TEST(OptimizationProjectionCostFunction, TestCreate_T) {
    Array2d const pixel{360, 240};
    Array3d const point{0, 0, 600};
    ceres::CostFunction const* const cost_function{optimization::Create_T<projection_functions::Pinhole>(pixel, point)};

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
