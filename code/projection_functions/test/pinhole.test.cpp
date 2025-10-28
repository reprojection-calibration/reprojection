#include <gtest/gtest.h>

#include "projection_functions/camera_model.hpp"

using namespace reprojection;

Array4d const pinhole_intrinsics{600, 600, 360, 240};
MatrixX3d const gt_points{{0, 0, 600},  //
                          {-360, 0, 600},
                          {360, 0, 600},
                          {0, -240, 600},
                          {0, 240, 600}};
// NOTE(Jack): I am not 100% sure if the pixels at x=720 should actually be part of the valid pixel group! These are
// technically one out of bounds based on the discretization of the pixels in the image. Our camera model currently
// does no bounds checking so this is not detected. If we add "valid pixel checking" these tests at the right and
// bottom might change as they are invalid pixels by one pixel.
MatrixX2d const gt_pixels{{pinhole_intrinsics[2], pinhole_intrinsics[3]},
                          {0, pinhole_intrinsics[3]},
                          {720, pinhole_intrinsics[3]},
                          {pinhole_intrinsics[2], 0},
                          {pinhole_intrinsics[2], 480}};

TEST(ProjectionFunctionsPinhole, TestPinholeProject) {
    auto const camera{projection_functions::PinholeCamera(pinhole_intrinsics)};
    MatrixX2d const pixels(camera.Project(gt_points));

    EXPECT_TRUE(pixels.isApprox(gt_pixels));
}

TEST(ProjectionFunctionsPinhole, TestPinholeUnproject) {
    auto const camera{projection_functions::PinholeCamera(pinhole_intrinsics)};
    MatrixX3d const rays{camera.Unproject(gt_pixels)};

    // Normalize the 3D points so we can compare them directly to the rays
    MatrixX3d const normalized_gt_points{gt_points.array() / 600};

    EXPECT_TRUE(rays.isApprox(normalized_gt_points));
}