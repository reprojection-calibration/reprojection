#include "projection_functions/unified_camera_model.hpp"

#include <gtest/gtest.h>

#include "types/eigen_types.hpp"

using namespace reprojection;

TEST(ProjectionFunctionsUnifiedCameraModel, TestUnifiedCameraModelProject) {
    Array5d const intrinsics{600, 600, 360, 240, 0.1};
    MatrixX3d const gt_points{{0, 0, 10}, {-360, 0, 600}, {360, 0, 600}, {0, -240, 600}, {0, 240, 600}};
    MatrixX2d const gt_pixels{{intrinsics[2], intrinsics[3]},
                              {37.598189013469323, intrinsics[3]},
                              {682.40181098653068, intrinsics[3]},
                              {intrinsics[2], 23.33548267325537},
                              {intrinsics[2], 456.66451732674466}};

    for (int i{0}; i < gt_points.rows(); ++i) {
        Eigen::Vector2d const pixel_i(
            projection_functions::UnifiedCameraModel::Project<double>(intrinsics, gt_points.row(i)));
        EXPECT_TRUE(pixel_i.isApprox(gt_pixels.row(i).transpose()));
    }
}
