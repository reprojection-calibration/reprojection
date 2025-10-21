#include "projection_functions/double_sphere.hpp"

#include <gtest/gtest.h>

using namespace reprojection;
using namespace reprojection::projection_functions;

Eigen::MatrixX3d const gt_points{{0, 0, 10}, {-360, 0, 600}, {360, 0, 600}, {0, -240, 600}, {0, 240, 600}};

TEST(ProjectionFunctionsDoubleSphere, TestDoubleSphereProjection) {
    Eigen::Array<double, 6, 1> const double_sphere_intrinsics{600, 600, 360, 240, 0.1, 0.2};

    // Heuristic groundtruth values caculated by running the projection functions itself once - hacky!
    Eigen::MatrixX2d const gt_pixels{{double_sphere_intrinsics[2], double_sphere_intrinsics[3]},
                                     {46.087794035716172, double_sphere_intrinsics[3]},
                                     {673.91220596428388, double_sphere_intrinsics[3]},
                                     {double_sphere_intrinsics[2], 26.040025446950779},
                                     {double_sphere_intrinsics[2], 453.95997455304922}};

    for (int i{0}; i < gt_points.rows(); ++i) {
        Eigen::Vector2d const pixel_i(
            DoubleSphereProjection<double>(double_sphere_intrinsics.data(), gt_points.row(i)));
        EXPECT_TRUE(pixel_i.isApprox(gt_pixels.row(i).transpose()));
    }
}

TEST(ProjectionFunctionsDoubleSphere, TestPinholeEquivalentProjection) {
    // If xi and alpha are zero then double sphere should essentially just act as a pinhole camera.
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