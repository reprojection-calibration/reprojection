#include "projection_functions/double_sphere.hpp"

#include <gtest/gtest.h>

#include "projection_functions/camera_model.hpp"

using namespace reprojection;
using namespace reprojection::projection_functions;

Eigen::Array<double, 6, 1> const double_sphere_intrinsics{600, 600, 360, 240, 0.1, 0.2};
Eigen::MatrixX3d const gt_points{{0, 0, 10}, {-360, 0, 600}, {360, 0, 600}, {0, -240, 600}, {0, 240, 600}};
Eigen::MatrixX2d const gt_pixels{{double_sphere_intrinsics[2], double_sphere_intrinsics[3]},
                                 {46.087794035716172, double_sphere_intrinsics[3]},
                                 {673.91220596428388, double_sphere_intrinsics[3]},
                                 {double_sphere_intrinsics[2], 26.040025446950779},
                                 {double_sphere_intrinsics[2], 453.95997455304922}};

TEST(ProjectionFunctionsDoubleSphere, TestDoubleSphereProject) {
    // GET RID OF LOOP HERE, USE BASE CLASS
    for (int i{0}; i < gt_points.rows(); ++i) {
        Eigen::Vector2d const pixel_i(DoubleSphere::Project<double>(double_sphere_intrinsics, gt_points.row(i)));
        EXPECT_TRUE(pixel_i.isApprox(gt_pixels.row(i).transpose()));
    }
}

TEST(ProjectionFunctionsDoubleSphere, TestPinholeEquivalentProjection) {
    // If xi and alpha are zero then double sphere should essentially just act as a pinhole camera.
    Eigen::Array<double, 6, 1> const pinhole_intrinsics{600, 600, 360, 240, 0, 0};

    Eigen::MatrixX2d const pinhole_pixels{{pinhole_intrinsics[2], pinhole_intrinsics[3]},
                                          {0, pinhole_intrinsics[3]},
                                          {720, pinhole_intrinsics[3]},
                                          {pinhole_intrinsics[2], 0},
                                          {pinhole_intrinsics[2], 480}};

    // GET RID OF LOOP HERE, USE BASE CLASS
    for (int i{0}; i < gt_points.rows(); ++i) {
        Eigen::Vector2d const pixel_i(DoubleSphere::Project<double>(pinhole_intrinsics, gt_points.row(i)));
        EXPECT_TRUE(pixel_i.isApprox(pinhole_pixels.row(i).transpose()));
    }
}

TEST(ProjectionFunctionsDoubleSphere, TestDoubleSphereUnproject) {
    auto const camera{projection_functions::DoubleSphereCamera(double_sphere_intrinsics)};
    MatrixX3d const rays{camera.Unproject(gt_pixels)};

    // NOTE(Jack): This is where the difference to a model like the pinhole model becomes apparent! For the pinhole
    // model all unprojected rays have a z-value of 1, and are somehow normalized in that sense. Here on the other hand,
    // for the double sphere model, all rays have a magnitude of one instead. Therefore instead of dividing the ground
    // truth points by the focal length like we did for pinhole unprojection, we here instead normalize each point into
    // a ray with magnitude one.
    MatrixX3d normalized_gt_points{gt_points};
    normalized_gt_points.rowwise().normalize();

    EXPECT_TRUE(rays.isApprox(normalized_gt_points));
}