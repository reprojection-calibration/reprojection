#include "projection_functions/camera_model.hpp"

#include <gtest/gtest.h>

#include "types/eigen_types.hpp"

using namespace reprojection;

TEST(ProjectionFunctionsCameraModel, TestPinholeCamera) {
    Array4d const intrinsics{600, 600, 360, 240};
    MatrixX3d const gt_points{{0, 0, 600},  //
                              {-360, 0, 600},
                              {360, 0, 600},
                              {0, -240, 600},
                              {0, 240, 600}};

    auto const camera{projection_functions::PinholeCamera(intrinsics)};
    MatrixX2d const pixels{camera.Project(gt_points)};
    MatrixX3d const rays{camera.Unproject(pixels)};

    EXPECT_EQ(pixels.rows(), gt_points.rows());
    EXPECT_EQ(rays.rows(), gt_points.rows());
}