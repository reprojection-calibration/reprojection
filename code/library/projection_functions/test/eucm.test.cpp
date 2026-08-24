#include "projection_functions/eucm.hpp"

#include <gtest/gtest.h>

#include "projection_functions/camera_model.hpp"
#include "testing_utilities/constants.hpp"
#include "types/calibration_types.hpp"
#include "types/eigen_types.hpp"

using namespace reprojection;

Array5d const intrinsics{600, 360, 240, 0.2, 0.1};
MatrixX2d const gt_pixels{{intrinsics[1], intrinsics[2]},
                          {1.27997, intrinsics[2]},
                          {718.621, intrinsics[2]},
                          {intrinsics[1], 0.381868},
                          {intrinsics[1], 479.519}};

TEST(ProjectionFunctionsEucm, TestEucmProject) {
    auto const camera{projection_functions::EucmCamera(intrinsics, testing_utilities::image_bounds)};

    auto const [pixels, mask](camera.Project(testing_utilities::gt_points));

    ASSERT_TRUE(mask.all());
    EXPECT_TRUE(pixels.isApprox(gt_pixels, 1e-3));
}

TEST(ProjectionFunctionsEucm, TestEucmUnproject) {
    auto const camera{projection_functions::EucmCamera(intrinsics, testing_utilities::image_bounds)};
    auto const [rays, mask]{camera.Unproject(gt_pixels)};

    // See note in double sphere test TestDoubleSphereUnproject about this normalization
    MatrixX3d normalized_gt_points{testing_utilities::gt_points};
    normalized_gt_points.rowwise().normalize();

    EXPECT_TRUE(rays.isApprox(normalized_gt_points, 1e-3));
    EXPECT_TRUE(mask.all());
}

TEST(ProjectionFunctionsEucm, TestEucmIntialize) {
    Array5d const result{projection_functions::Eucm::Initialize(1200, 480, 720)};
    Array5d const gt_result{1200, 360, 240, 0.5, 1};

    EXPECT_TRUE(result.isApprox(gt_result));
}