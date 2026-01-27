#include "projection_functions/pinhole.hpp"

#include <gtest/gtest.h>

#include "projection_functions/camera_model.hpp"
#include "testing_utilities/constants.hpp"
#include "types/calibration_types.hpp"
#include "types/eigen_types.hpp"

using namespace reprojection;

// TODO(Jack): Add a test for all cameras on a single pixel NOT using the templated Eigen projection/unprojection.

MatrixX3d const gt_points{{0, 0, 600},  //
                          {-360, 0, 600},
                          {359.9, 0, 600},
                          {0, -240, 600},
                          {0, 239.9, 600}};
MatrixX2d const gt_pixels{{360, 240},  //
                          {0, 240},
                          {719.9, 240},
                          {360, 0},
                          {360, 479.9}};

TEST(ProjectionFunctionsPinhole, TestPinholeProject) {
    auto const camera{
        projection_functions::PinholeCamera(testing_utilities::pinhole_intrinsics, testing_utilities::image_bounds)};
    auto const [pixels, mask](camera.Project(gt_points));

    // NOTE(Jack): We assert that all pixels are valid because if this is not true that means a fundamental assumption
    // is broken. The test data was engineered to meet this condition. In a real application you should mask our the
    // pixels to catch invalid ones, but not here in the test case.
    ASSERT_TRUE(mask.all());
    EXPECT_TRUE(pixels.isApprox(gt_pixels));
}

TEST(ProjectionFunctionsPinhole, TestPinholeProjectMasking) {
    // Point in front of camera - returns pixel.
    auto pixel{projection_functions::Pinhole::Project(testing_utilities::pinhole_intrinsics,
                                                      testing_utilities::image_bounds, {0, 0, 10})};
    EXPECT_TRUE(pixel.has_value());

    // Point behind camera - returns std::nullopt.
    pixel = projection_functions::Pinhole::Project(testing_utilities::pinhole_intrinsics,
                                                   testing_utilities::image_bounds, {0, 0, -10});
    EXPECT_FALSE(pixel.has_value());

    // Point that project to way outside the bounds - returns std::nullopt.
    pixel = projection_functions::Pinhole::Project(testing_utilities::pinhole_intrinsics,
                                                   testing_utilities::image_bounds, {100, 100, 10});
    EXPECT_FALSE(pixel.has_value());
}

TEST(ProjectionFunctionsPinhole, TestPinholeUnproject) {
    auto const camera{
        projection_functions::PinholeCamera(testing_utilities::pinhole_intrinsics, testing_utilities::image_bounds)};
    MatrixX3d const rays{camera.Unproject(gt_pixels)};

    // Multiply rays by metric scale to put them back into world coordinates
    EXPECT_TRUE((600 * rays).isApprox(gt_points));
}