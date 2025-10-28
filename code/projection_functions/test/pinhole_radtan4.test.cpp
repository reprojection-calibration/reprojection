#include "projection_functions/pinhole_radtan4.hpp"

#include <gtest/gtest.h>

#include "projection_functions/camera_model.hpp"
#include "types/eigen_types.hpp"

using namespace reprojection;

Eigen::Array<double, 8, 1> const pinhole_radtan4_intrinsics{600, 600, 360, 240, -0.1, 0.1, 0.001, 0.001};
Eigen::MatrixX3d const gt_points{{0, 0, 600},  //
                                 {-360, 0, 600},
                                 {360, 0, 600},
                                 {0, -240, 600},
                                 {0, 240, 600}};
Eigen::MatrixX2d const gt_pixels{{pinhole_radtan4_intrinsics[2], pinhole_radtan4_intrinsics[3]},
                                 {8.9424000000000206, 240.21600000000001},
                                 {712.35359999999991, 240.21600000000001},
                                 {360.096, 3.5135999999999683},
                                 {360.096, 477.06240000000003}};

TEST(ProjectionFunctionsPinholeRadtan4, TestProject) {
    auto const camera{projection_functions::PinholeRadtan4Camera(pinhole_radtan4_intrinsics)};
    MatrixX2d const pixels(camera.Project(gt_points));

    EXPECT_TRUE(pixels.isApprox(gt_pixels));
}

TEST(ProjectionFunctionsPinholeRadtan4, TestPinholeEquivalentProject) {
    // If [k1, k2, p1, p2] are zero then pinhole radtan4 should essentially just act as a pinhole camera.
    Eigen::Array<double, 8, 1> const pinhole_intrinsics{600, 600, 360, 240, 0, 0, 0, 0};
    Eigen::MatrixX2d const gt_pinhole_pixels{{pinhole_intrinsics[2], pinhole_intrinsics[3]},
                                             {0, pinhole_intrinsics[3]},
                                             {720, pinhole_intrinsics[3]},
                                             {pinhole_intrinsics[2], 0},
                                             {pinhole_intrinsics[2], 480}};

    auto const camera{projection_functions::PinholeRadtan4Camera(pinhole_intrinsics)};
    MatrixX2d const pixels(camera.Project(gt_points));

    EXPECT_TRUE(pixels.isApprox(gt_pinhole_pixels));
}

TEST(ProjectionFunctionsPinholeRadtan4, TestUnproject) {
    auto const camera{projection_functions::PinholeRadtan4Camera(pinhole_radtan4_intrinsics)};
    MatrixX3d const rays(camera.Unproject(gt_pixels));

    // Normalize the 3D points so we can compare them directly to the rays
    MatrixX3d const normalized_gt_points{gt_points.array() / 600};

    EXPECT_TRUE(rays.isApprox(normalized_gt_points));
}

TEST(ProjectionFunctionsPinholeRadtan4, TestJacobianUpdate) {
    Eigen::Array<double, 4, 1> const distortion{-0.1, 0.1, 0.001, 0.001};
    Eigen::Array2d const p_cam{-0.1, -0.1};
    auto const [distorted_p_cam, J]{projection_functions::PinholeRadtan4::JacobianUpdate(distortion, p_cam)};

    EXPECT_FLOAT_EQ(distorted_p_cam[0], -0.099743999999999999);
    EXPECT_FLOAT_EQ(distorted_p_cam[1], -0.099743999999999999);
    EXPECT_FLOAT_EQ(J(0, 0), 0.99531999999999998);
    EXPECT_FLOAT_EQ(J(0, 1), -0.0023200000000000004);
    EXPECT_FLOAT_EQ(J(1, 0), -0.0023200000000000004);
    EXPECT_FLOAT_EQ(J(1, 1), 0.99531999999999998);
}

TEST(ProjectionFunctionsPinholeRadtan4, TestDistortionFunctor) {
    Eigen::Array<double, 4, 1> const distortion{-0.1, 0.1, 0.001, 0.001};
    auto const distortion_functor{projection_functions::PinholeRadtan4::DistortFunctor(distortion)};

    Eigen::Array2d const p_cam{-0.1, -0.1};
    Eigen::Array2d distorted_p_cam;
    distortion_functor(p_cam.data(), distorted_p_cam.data());

    EXPECT_FLOAT_EQ(distorted_p_cam[0], -0.099743999999999999);
    EXPECT_FLOAT_EQ(distorted_p_cam[1], -0.099743999999999999);
}
