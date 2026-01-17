#include "projection_functions/pinhole.hpp"

#include <gtest/gtest.h>

#include "projection_functions/camera_model.hpp"
#include "projection_functions/image_bounds.hpp"
#include "types/eigen_types.hpp"

using namespace reprojection;

// TODO(Jack): Add a test for all cameras on a single pixel NOT using the templated Eigen projection/unprojection.

Array4d const pinhole_intrinsics{600, 600, 360, 240};
projection_functions::ImageBounds const bounds{{0, 720, 0, 480}};
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
    auto const camera{projection_functions::PinholeCamera(pinhole_intrinsics, bounds)};
    auto const [pixels, mask](camera.Project(gt_points));

    // NOTE(Jack): We assert that all pixels are valid because if this is not true that means a fundamental assumption
    // is broken. The test data was engineered to meet this condition. In a real application you should mask our the
    // pixels to catch invalid ones, but not here in the test case.
    ASSERT_TRUE(mask.all());
    EXPECT_TRUE(pixels.isApprox(gt_pixels));
}

TEST(ProjectionFunctionsPinhole, TestPinholeProjectMasking) {
    // Point in front of camera - returns pixel.
    auto pixel{projection_functions::Pinhole::Project(pinhole_intrinsics, bounds, {0, 0, 10})};
    EXPECT_TRUE(pixel.has_value());

    // Point behind camera - returns std::nullopt.
    pixel = projection_functions::Pinhole::Project(pinhole_intrinsics, bounds, {0, 0, -10});
    EXPECT_FALSE(pixel.has_value());
}

TEST(ProjectionFunctionsPinhole, TestPinholeUnproject) {
    auto const camera{projection_functions::PinholeCamera(pinhole_intrinsics, bounds)};
    MatrixX3d const rays{camera.Unproject(gt_pixels)};

    // Multiply rays by metric scale to put them back into world coordinates
    EXPECT_TRUE((600 * rays).isApprox(gt_points));
}