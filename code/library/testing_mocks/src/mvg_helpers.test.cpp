#include "mvg_helpers.hpp"

#include <gtest/gtest.h>

#include "testing_utilities/constants.hpp"
#include "types/eigen_types.hpp"

using namespace reprojection;

TEST(TestingMocksMvgGenerator, TestProject) {
    MatrixX3d const points_w{{0.00, 0.00, 5.00},   {1.00, 1.00, 5.00},   {-1.00, -1.00, 5.00},
                             {2.00, -1.00, 10.00}, {-2.00, 1.00, 10.00}, {0.50, -0.50, 7.00}};
    MatrixX2d const gt_pixels{{360.00, 240.00}, {480.00, 360.00}, {240.00, 120.00},
                              {480.00, 180.00}, {240.00, 300.00}, {402.857, 197.144}};

    auto const camera{std::unique_ptr<projection_functions::Camera>(new projection_functions::PinholeCamera(
        testing_utilities::pinhole_intrinsics, testing_utilities::image_bounds))};
    Isometry3d const tf_co_w{Isometry3d::Identity()};

    auto const [pixels, mask]{testing_mocks::MvgHelpers::Project(points_w, camera, tf_co_w)};
    ASSERT_TRUE(mask.all());
    EXPECT_TRUE(pixels.isApprox(gt_pixels, 1e-3));
}

TEST(TestingMocksMvgGenerator, TestProjectMasking) {
    // Given the tf_co_w transform set below, the last point here will be behind the camera and should be masked out!
    MatrixX3d const points_w{{0.00, 0.00, 10.00},  //
                             {1.00, 1.00, 10.00},
                             {-1.00, -1.00, 5.00}};
    Array3<bool> const gt_mask{true, true, false};

    auto const camera{std::unique_ptr<projection_functions::Camera>(new projection_functions::PinholeCamera(
        testing_utilities::pinhole_intrinsics, testing_utilities::image_bounds))};
    Isometry3d tf_co_w{Isometry3d::Identity()};
    tf_co_w.translation().z() = -6.0;

    auto const [pixels, mask]{testing_mocks::MvgHelpers::Project(points_w, camera, tf_co_w)};
    ASSERT_TRUE(mask.isApprox(gt_mask));
}
