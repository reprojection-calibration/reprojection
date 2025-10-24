#include "projection_functions/pinhole.hpp"

#include <gtest/gtest.h>

#include "eigen_utilities/camera.hpp"

using namespace reprojection;
using namespace reprojection::projection_functions;

Eigen::Array<double, 4, 1> const pinhole_intrinsics{600, 600, 360, 240};
Eigen::MatrixX3d const gt_points{{0, 0, 600}, {-360, 0, 600}, {360, 0, 600}, {0, -240, 600}, {0, 240, 600}};
// NOTE(Jack): I am not 100% sure if the pixels at x=720 should actually be part of the valid pixel group! These are
// technically one out of bounds based on the discretization of the pixels in the image. Our camera model currently
// does no bounds checking so this is not detected. If we add "valid pixel checking" these tests at the right and
// bottom might change as they are invalid pixels by one pixel.
Eigen::MatrixX2d const gt_pixels{{pinhole_intrinsics[2], pinhole_intrinsics[3]},
                                 {0, pinhole_intrinsics[3]},
                                 {720, pinhole_intrinsics[3]},
                                 {pinhole_intrinsics[2], 0},
                                 {pinhole_intrinsics[2], 480}};

TEST(ProjectionFunctionsPinhole, TestPinholeProjection) {
    Eigen::MatrixX2d const pixels(PinholeProjection(eigen_utilities::ToK(pinhole_intrinsics), gt_points));

    EXPECT_TRUE(pixels.isApprox(gt_pixels));
}

TEST(ProjectionFunctionsPinhole, TestPinholeUnprojection) {
    for (int i{0}; i < gt_pixels.rows(); i++) {
        Eigen::Vector3d const ray_i{PinholeUnprojection<double>(pinhole_intrinsics, gt_pixels.row(i).array())};
        EXPECT_TRUE(ray_i.isApprox(gt_points.row(i).transpose() / 600));  // Divide by focal length
    }
}