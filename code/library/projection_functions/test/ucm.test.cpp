#include "projection_functions/ucm.hpp"

#include <gtest/gtest.h>

#include "projection_functions/camera_model.hpp"
#include "testing_utilities/constants.hpp"
#include "types/calibration_types.hpp"
#include "types/eigen_types.hpp"

using namespace reprojection;

Array4d const intrinsics{600, 360, 240, 0.5};
MatrixX2d const gt_pixels{{intrinsics[1], intrinsics[2]},
                          {27.6192, intrinsics[2]},
                          {692.302, intrinsics[2]},
                          {intrinsics[1], 8.90112},
                          {intrinsics[1], 471.009}};

TEST(ProjectionFunctionsUcm, TestUcmProject) {
    auto const camera{projection_functions::UcmCamera(intrinsics, testing_utilities::image_bounds)};

    auto const [pixels, mask](camera.Project(testing_utilities::gt_points));

    ASSERT_TRUE(mask.all());
    EXPECT_TRUE(pixels.isApprox(gt_pixels, 1e-3));
}

// TODO(Jack): Add a test for invalid unprojection
TEST(ProjectionFunctionsUcm, TestUcmUnproject) {
    auto const camera{projection_functions::UcmCamera(intrinsics, testing_utilities::image_bounds)};
    auto const [rays, mask]{camera.Unproject(gt_pixels)};

    // See note in double sphere test TestDoubleSphereUnproject about this normalization
    MatrixX3d normalized_gt_points{testing_utilities::gt_points};
    normalized_gt_points.rowwise().normalize();

    EXPECT_TRUE(rays.isApprox(normalized_gt_points, 1e-3));
    EXPECT_TRUE(mask.all());
}

TEST(ProjectionFunctionsUcm, TestUcmIntialize) {
    Array4d const result{projection_functions::Ucm::Initialize(1200, 480, 720)};
    Array4d const gt_result{1200, 360, 240, 0.5};

    EXPECT_TRUE(result.isApprox(gt_result));
}