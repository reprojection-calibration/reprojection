#include "projection_functions/pinhole_radtan4.hpp"

#include <gtest/gtest.h>

#include "types/eigen_types.hpp"

using namespace reprojection;
using namespace reprojection::projection_functions;

Eigen::Array<double, 8, 1> const pinhole_radtan4_intrinsics{600, 600, 360, 240, -0.1, 0.1, 0.001, 0.001};
Eigen::MatrixX3d const gt_points{{0, 0, 600}, {-360, 0, 600}, {360, 0, 600}, {0, -240, 600}, {0, 240, 600}};
// Heuristic groundtruth values caculated by running the projection functions itself once - hacky!
Eigen::MatrixX2d const gt_pixels{{pinhole_radtan4_intrinsics[2], pinhole_radtan4_intrinsics[3]},
                                 {8.9424000000000206, 240.21600000000001},
                                 {712.35359999999991, 240.21600000000001},
                                 {360.096, 3.5135999999999683},
                                 {360.096, 477.06240000000003}};

TEST(ProjectionFunctionsPinholeRadtan4, TestPinholeRadtan4Projection) {
    for (int i{0}; i < gt_points.rows(); ++i) {
        Eigen::Vector2d const pixel_i(PinholeRadtan4Projection<double>(pinhole_radtan4_intrinsics, gt_points.row(i)));
        EXPECT_TRUE(pixel_i.isApprox(gt_pixels.row(i).transpose()));
    }
}

TEST(ProjectionFunctionsPinholeRadtan4, TestPinholeEquivalentProjection) {
    // If [k1, k2, p1, p2] are zero then pinhole radtan4 should essentially just act as a pinhole camera.
    Eigen::Array<double, 8, 1> const pinhole_intrinsics{600, 600, 360, 240, 0, 0, 0, 0};
    Eigen::MatrixX2d const gt_pinhole_pixels{{pinhole_intrinsics[2], pinhole_intrinsics[3]},
                                             {0, pinhole_intrinsics[3]},
                                             {720, pinhole_intrinsics[3]},
                                             {pinhole_intrinsics[2], 0},
                                             {pinhole_intrinsics[2], 480}};

    for (int i{0}; i < gt_points.rows(); ++i) {
        Eigen::Vector2d const pixel_i(PinholeRadtan4Projection<double>(pinhole_intrinsics, gt_points.row(i)));
        EXPECT_TRUE(pixel_i.isApprox(gt_pinhole_pixels.row(i).transpose()));
    }
}

TEST(ProjectionFunctionsPinholeRadtan4, TestPinholeRadtan4Unprojection) {
    for (int i{0}; i < gt_pixels.rows(); i++) {
        Eigen::Vector3d const ray_i{
            PinholeRadtan4Unprojection<double>(pinhole_radtan4_intrinsics, gt_pixels.row(i).array())};
        EXPECT_TRUE(ray_i.isApprox(gt_points.row(i).transpose() / 600));  // Divide by focal length
    }
}

TEST(ProjectionFunctionsPinholeRadtan4, TestRadtan4DistortionJacobianUpdate) {
    Eigen::Array<double, 4, 1> const distortion{-0.1, 0.1, 0.001, 0.001};
    Eigen::Array2d const p_cam{-0.1, -0.1};
    auto const [distorted_p_cam, J]{Radtan4DistortionJacobianUpdate(distortion, p_cam)};

    EXPECT_FLOAT_EQ(distorted_p_cam[0], -0.099743999999999999);
    EXPECT_FLOAT_EQ(distorted_p_cam[1], -0.099743999999999999);
    EXPECT_FLOAT_EQ(J(0, 0), 0.99531999999999998);
    EXPECT_FLOAT_EQ(J(0, 1), -0.0023200000000000004);
    EXPECT_FLOAT_EQ(J(1, 0), -0.0023200000000000004);
    EXPECT_FLOAT_EQ(J(1, 1), 0.99531999999999998);
}
