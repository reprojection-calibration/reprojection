#include "projection_functions/unified_camera_model.hpp"

#include <gtest/gtest.h>

#include "projection_functions/camera_model.hpp"
#include "types/eigen_types.hpp"

using namespace reprojection;

Array5d const intrinsics{600, 600, 360, 240, 0.1};
MatrixX3d const gt_points{{0, 0, 10},  //
                          {-360, 0, 600},
                          {360, 0, 600},
                          {0, -240, 600},
                          {0, 240, 600}};
MatrixX2d const gt_pixels{{intrinsics[2], intrinsics[3]},
                          {37.598189013469323, intrinsics[3]},
                          {682.40181098653068, intrinsics[3]},
                          {intrinsics[2], 23.33548267325537},
                          {intrinsics[2], 456.66451732674466}};

TEST(ProjectionFunctionsUnifiedCameraModel, TestUnifiedCameraModelProject) {
    auto const camera{projection_functions::UcmCamera(intrinsics)};
    MatrixX2d const pixels(camera.Project(gt_points));

    EXPECT_TRUE(pixels.isApprox(gt_pixels));
}

TEST(ProjectionFunctionsUnifiedCameraModel, TestUnifiedCameraModelUnproject) {
    auto const camera{projection_functions::UcmCamera(intrinsics)};
    MatrixX3d const rays{camera.Unproject(gt_pixels)};

    // See note in double sphere testing about this normalization
    MatrixX3d normalized_gt_points{gt_points};
    normalized_gt_points.rowwise().normalize();

    EXPECT_TRUE(rays.isApprox(normalized_gt_points));
}
