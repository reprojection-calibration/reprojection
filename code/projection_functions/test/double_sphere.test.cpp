#include "projection_functions/double_sphere.hpp"

#include <gtest/gtest.h>

#include "eigen_utilities/camera.hpp"

using namespace reprojection;
using namespace reprojection::projection_functions;

Eigen::MatrixX3d const gt_points{{0, 0, 10}, {-360, 0, 600}, {360, 0, 600}, {0, -240, 600}, {0, 240, 600}};
// NOTE(Jack): I am not 100% sure if the pixels at x=720 should actually be part of the valid pixel group! These are
// technically one out of bounds based on the discretization of the pixels in the image. Our camera model currently
// does no bounds checking so this is not detected. If we add "valid pixel checking" these tests at the right and
// bottom might change as they are invalid pixels by one pixel.

TEST(ProjectionFunctionsDoubleSphere, TestDoubleSphereProjection) {
    Eigen::Array<double, 6, 1> const double_sphere_intrinsics{600, 600, 360, 240, 0.1, 0.2};

    Eigen::MatrixX2d const gt_pixels{{double_sphere_intrinsics[2], double_sphere_intrinsics[3]},
                                     {46.0878, double_sphere_intrinsics[3]},
                                     {673.912, double_sphere_intrinsics[3]},
                                     {double_sphere_intrinsics[2], 26.04},
                                     {double_sphere_intrinsics[2], 453.96}};

    for (int i{0}; i < gt_points.rows(); ++i) {
        Eigen::Vector2d const pixel_i(
            DoubleSphereProjection<double>(double_sphere_intrinsics.data(), gt_points.row(i)));

        EXPECT_TRUE(pixel_i.isApprox(gt_pixels.row(i).transpose()));
    }
}

TEST(ProjectionFunctionsDoubleSphere, TestPinholeEquivalentProjection) {
    Eigen::Array<double, 6, 1> const pinhole_intrinsics{600, 600, 360, 240, 0, 0};

    Eigen::MatrixX2d const gt_pixels{{pinhole_intrinsics[2], pinhole_intrinsics[3]},
                                     {0, pinhole_intrinsics[3]},
                                     {720, pinhole_intrinsics[3]},
                                     {pinhole_intrinsics[2], 0},
                                     {pinhole_intrinsics[2], 480}};

    for (int i{0}; i < gt_points.rows(); ++i) {
        Eigen::Vector2d const pixel_i(DoubleSphereProjection<double>(pinhole_intrinsics.data(), gt_points.row(i)));

        EXPECT_TRUE(pixel_i.isApprox(gt_pixels.row(i).transpose()));
    }
}