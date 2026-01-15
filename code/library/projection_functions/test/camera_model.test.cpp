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
    MatrixX2d const gt_pixels{{360, 240},  //
                              {0, 240},
                              {720, 240},
                              {360, 0},
                              {360, 480}};

    auto const camera{projection_functions::PinholeCamera(intrinsics)};

    auto const [pixels, mask]{camera.Project(gt_points)};
    ASSERT_TRUE(mask.all());
    EXPECT_TRUE(pixels.isApprox(gt_pixels));

    // NOTE(Jack): Of course pinhole unprojection looses the scale, and we are just returned rays in space with z=1.
    // Luckily for the test case we know the scale (600) and can therefore transform the rays into points and compare
    // them directly against the groundtruth input points.
    // WARN(Jack): In most real applications we would want to mask the returned pixels to make sure we only unproject
    // valid pixels. However, in this highly controlled and engineered test case we do not need to do that. All real
    // application should!
    MatrixX3d const rays{camera.Unproject(pixels)};
    EXPECT_TRUE((600 * rays).isApprox(gt_points));
}

// WARN(Jack): At time of writing the masking only handles points behind the camera! It does not handle if z=0 or the
// general field of view limits. These are both additions which should be added to the pinhole projection mask.
// TODO(Jack): Add and test unprojection masking!
TEST(ProjectionFunctionsCameraModel, TestPinholeCameraProjectionMasking) {
    Array4d const intrinsics{600, 600, 360, 240};
    MatrixX3d const gt_points{{0, 0, -600},  //
                              {0, 0, 600},
                              {0, 0, -600},
                              {0, 0, -600},
                              {0, 0, 600}};
    Array5b const gt_mask{false, true, false, false, true};

    auto const camera{projection_functions::PinholeCamera(intrinsics)};

    auto const [_, mask]{camera.Project(gt_points)};
    EXPECT_TRUE(mask.isApprox(gt_mask));
}