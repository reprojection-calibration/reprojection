#include "projection_functions/pinhole.hpp"

#include <gtest/gtest.h>

using namespace reprojection::projection_functions;

TEST(ProjectionFunctionsPinhole, TestPinholeProjection) {
    Eigen::Array<double, 4, 1> const pinhole_intrinsics{600, 600, 360, 240};

    Eigen::Vector3d const center_point{0, 0, 10};
    Eigen::Vector2d const center_pixel{PinholeProjection(pinhole_intrinsics.data(), center_point)};
    EXPECT_TRUE(center_pixel.isApprox(Eigen::Vector2d{pinhole_intrinsics[2], pinhole_intrinsics[3]}));

    Eigen::Vector3d const left_point{-360, 0, 600};
    Eigen::Vector2d const left_pixel{PinholeProjection(pinhole_intrinsics.data(), left_point)};
    EXPECT_TRUE(left_pixel.isApprox(Eigen::Vector2d{0, pinhole_intrinsics[3]}));

    // NOTE(Jack): I am not 100% sure if the pixels at x=720 should actually be part of the valid pixel group! These are
    // technically one out of bounds based on the discretization of the pixels in the image. Our camera model currently
    // does no bounds checking so this is not detected. If we add "valid pixel checking" these tests at the right and
    // bottom might change as they are invalid pixels by one pixel.
    Eigen::Vector3d const right_point{360, 0, 600};
    Eigen::Vector2d const right_pixel{PinholeProjection(pinhole_intrinsics.data(), right_point)};
    EXPECT_TRUE(right_pixel.isApprox(Eigen::Vector2d{720, pinhole_intrinsics[3]}));

    Eigen::Vector3d const top_point{0, -240, 600};
    Eigen::Vector2d const top_pixel{PinholeProjection(pinhole_intrinsics.data(), top_point)};
    EXPECT_TRUE(top_pixel.isApprox(Eigen::Vector2d{pinhole_intrinsics[2], 0}));

    Eigen::Vector3d const bottom_point{0, 240, 600};
    Eigen::Vector2d const bottom_pixel{PinholeProjection(pinhole_intrinsics.data(), bottom_point)};
    EXPECT_TRUE(bottom_pixel.isApprox(Eigen::Vector2d{pinhole_intrinsics[2], 480}));
}