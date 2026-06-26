#include "reprojection_error_spline.hpp"

#include <gtest/gtest.h>

#include "projection_functions/double_sphere.hpp"
#include "projection_functions/pinhole.hpp"
#include "projection_functions/pinhole_radtan4.hpp"
#include "projection_functions/unified_camera_model.hpp"
#include "testing_utilities/constants.hpp"

using namespace reprojection;
using namespace reprojection::optimization::cost_functions;

TEST(OptimizationCostFunctions, TestReprojectionErrorSplineCreate) {
    Array2d const pixel{360, 240};
    Array3d const point{0, 0, 600};
    double const u_i{0};
    uint64_t const delta_t_ns{1};

    int const num_parameter_blocks{5};  // intrinsics and four control points

    ceres::CostFunction* cost_function{
        Create(CameraModel::DoubleSphere, testing_utilities::image_bounds, pixel, point, u_i, delta_t_ns)};
    EXPECT_EQ(std::size(cost_function->parameter_block_sizes()), num_parameter_blocks);
    EXPECT_EQ(cost_function->parameter_block_sizes()[0], projection_functions::DoubleSphere::Size);
    delete cost_function;

    cost_function = Create(CameraModel::Pinhole, testing_utilities::image_bounds, pixel, point, u_i, delta_t_ns);
    EXPECT_EQ(std::size(cost_function->parameter_block_sizes()), num_parameter_blocks);
    EXPECT_EQ(cost_function->parameter_block_sizes()[0], projection_functions::Pinhole::Size);
    delete cost_function;

    cost_function = Create(CameraModel::PinholeRadtan4, testing_utilities::image_bounds, pixel, point, u_i, delta_t_ns);
    EXPECT_EQ(std::size(cost_function->parameter_block_sizes()), num_parameter_blocks);
    EXPECT_EQ(cost_function->parameter_block_sizes()[0], projection_functions::PinholeRadtan4::Size);
    delete cost_function;

    cost_function =
        Create(CameraModel::UnifiedCameraModel, testing_utilities::image_bounds, pixel, point, u_i, delta_t_ns);
    EXPECT_EQ(std::size(cost_function->parameter_block_sizes()), num_parameter_blocks);
    EXPECT_EQ(cost_function->parameter_block_sizes()[0], projection_functions::UnifiedCameraModel::Size);
    delete cost_function;
}

// Test with four of the same identity control points. This should give us an identity SE3 pose. Because the spline is
// stationary as identity we arbitrarily choose u_i and delta_t_ns as it makes no difference here.
TEST(OptimizationCostFunctions, TestReprojectionErrorSpline_T) {
    Array2d const pixel{testing_utilities::pinhole_intrinsics[1], testing_utilities::pinhole_intrinsics[2]};
    Array3d const point{0, 0, 10};
    ReprojectionErrorSpline_T<projection_functions::Pinhole> const cost_function{pixel, point,
                                                                                 testing_utilities::image_bounds, 0, 1};

    Array6d const control_point{Array6d::Zero()};
    Array2d residual{-1, -1};
    bool const success{cost_function(testing_utilities::pinhole_intrinsics.data(), control_point.data(),
                                     control_point.data(), control_point.data(), control_point.data(),
                                     residual.data())};
    EXPECT_TRUE(success);
    EXPECT_FLOAT_EQ(residual[0], 0.0);
    EXPECT_FLOAT_EQ(residual[1], 0.0);
}

TEST(OptimizationCostFunctions, TestReprojectionErrorSpline_TCreate) {
    Array2d const pixel{360, 240};
    Array3d const point{0, 0, 600};
    ceres::CostFunction const* const cost_function{ReprojectionErrorSpline_T<projection_functions::Pinhole>::Create(
        pixel, point, testing_utilities::image_bounds, 0.0, 1)};

    EXPECT_EQ(std::size(cost_function->parameter_block_sizes()), 5);
    EXPECT_EQ(cost_function->parameter_block_sizes()[0], 3);  // pinhole intrinsics
    EXPECT_EQ(cost_function->parameter_block_sizes()[1], 6);  // control point 1
    EXPECT_EQ(cost_function->parameter_block_sizes()[2], 6);  // control point 2
    EXPECT_EQ(cost_function->parameter_block_sizes()[3], 6);  // control point 3
    EXPECT_EQ(cost_function->parameter_block_sizes()[4], 6);  // control point 4
    EXPECT_EQ(cost_function->num_residuals(), 2);
    delete cost_function;
}