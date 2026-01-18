#include "projection_functions/pinhole_radtan4.hpp"

#include <gtest/gtest.h>

#include "projection_functions/camera_model.hpp"
#include "types/calibration_types.hpp"
#include "types/eigen_types.hpp"

using namespace reprojection;

Array8d const pinhole_radtan4_intrinsics{600, 600, 360, 240, -0.1, 0.1, 0.001, 0.001};
ImageBounds const bounds{0, 720, 0, 480};
MatrixX3d const gt_points{{0, 0, 600},  //
                          {-360, 0, 600},
                          {359.9, 0, 600},
                          {0, -240, 600},
                          {0, 239.9, 600}};
MatrixX2d const gt_pixels{{pinhole_radtan4_intrinsics[2], pinhole_radtan4_intrinsics[3]},
                          {8.9424000000000206, 240.21600000000001},
                          {712.25756064927782, 240.21588001666666},
                          {360.096, 3.5135999999999683},
                          {360.09592001666664, 476.96567911650004}};

TEST(ProjectionFunctionsPinholeRadtan4, TestProject) {
    auto const camera{projection_functions::PinholeRadtan4Camera(pinhole_radtan4_intrinsics, bounds)};

    auto const [pixels, mask](camera.Project(gt_points));
    ASSERT_TRUE(mask.all());
    EXPECT_TRUE(pixels.isApprox(gt_pixels));
}

TEST(ProjectionFunctionsPinholeRadtan4, TestPinholeEquivalentProject) {
    // If [k1, k2, p1, p2] are zero then pinhole radtan4 should essentially just act as a pinhole camera.
    Array8d const pinhole_intrinsics{600, 600, 360, 240, 0, 0, 0, 0};
    MatrixX2d const gt_pinhole_pixels{{360, 240},  //
                                      {0, 240},
                                      {719.9, 240},
                                      {360, 0},
                                      {360, 479.9}};

    auto const camera{projection_functions::PinholeRadtan4Camera(pinhole_intrinsics, bounds)};

    auto const [pixels, mask](camera.Project(gt_points));
    ASSERT_TRUE(mask.all());
    EXPECT_TRUE(pixels.isApprox(gt_pinhole_pixels));
}

// TODO(Jack): Test masking!

TEST(ProjectionFunctionsPinholeRadtan4, TestUnproject) {
    auto const camera{projection_functions::PinholeRadtan4Camera(pinhole_radtan4_intrinsics, bounds)};
    MatrixX3d const rays(camera.Unproject(gt_pixels));

    // Normalize the 3D points so we can compare them directly to the rays
    MatrixX3d const normalized_gt_points{gt_points.array() / 600};

    EXPECT_TRUE(rays.isApprox(normalized_gt_points));
}

TEST(ProjectionFunctionsPinholeRadtan4, TestJacobianUpdate) {
    Array4d const distortion{-0.1, 0.1, 0.001, 0.001};
    Array2d const p_cam{-0.1, -0.1};
    auto const [distorted_p_cam, J]{projection_functions::PinholeRadtan4::JacobianUpdate(distortion, p_cam)};

    EXPECT_FLOAT_EQ(distorted_p_cam[0], -0.099743999999999999);
    EXPECT_FLOAT_EQ(distorted_p_cam[1], -0.099743999999999999);
    EXPECT_FLOAT_EQ(J(0, 0), 0.99531999999999998);
    EXPECT_FLOAT_EQ(J(0, 1), -0.0023200000000000004);
    EXPECT_FLOAT_EQ(J(1, 0), -0.0023200000000000004);
    EXPECT_FLOAT_EQ(J(1, 1), 0.99531999999999998);
}

TEST(ProjectionFunctionsPinholeRadtan4, TestDistortionFunctor) {
    Array4d const distortion{-0.1, 0.1, 0.001, 0.001};
    auto const distortion_functor{projection_functions::PinholeRadtan4::DistortFunctor(distortion)};

    Array2d const p_cam{-0.1, -0.1};
    Array2d distorted_p_cam;
    distortion_functor(p_cam.data(), distorted_p_cam.data());

    EXPECT_FLOAT_EQ(distorted_p_cam[0], -0.099743999999999999);
    EXPECT_FLOAT_EQ(distorted_p_cam[1], -0.099743999999999999);
}
