#include "projection_functions/unified_camera_model.hpp"

#include <gtest/gtest.h>

#include "projection_functions/camera_model.hpp"
#include "testing_utilities/constants.hpp"
#include "types/calibration_types.hpp"
#include "types/eigen_types.hpp"

using namespace reprojection;

Array5d const intrinsics{600, 600, 360, 240, 0.1};
MatrixX2d const gt_pixels{{intrinsics[2], intrinsics[3]},
                          {37.598189013469323, intrinsics[3]},
                          {682.31472984876814, intrinsics[3]},
                          {intrinsics[2], 23.33548267325537},
                          {intrinsics[2], 456.57545045050017}};

TEST(ProjectionFunctionsUnifiedCameraModel, TestUnifiedCameraModelProject) {
    auto const camera{projection_functions::UcmCamera(intrinsics, testing_utilities::image_bounds)};

    auto const [pixels, mask](camera.Project(testing_utilities::gt_points));
    ASSERT_TRUE(mask.all());
    EXPECT_TRUE(pixels.isApprox(gt_pixels));
}

TEST(ProjectionFunctionsUnifiedCameraModel, TestUnifiedCameraModelUnproject) {
    auto const camera{projection_functions::UcmCamera(intrinsics, testing_utilities::image_bounds)};
    MatrixX3d const rays{camera.Unproject(gt_pixels)};

    // See note in double sphere test TestDoubleSphereUnproject about this normalization
    MatrixX3d normalized_gt_points{testing_utilities::gt_points};
    normalized_gt_points.rowwise().normalize();

    EXPECT_TRUE(rays.isApprox(normalized_gt_points));
}
