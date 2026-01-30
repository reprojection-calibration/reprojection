#include "testing_mocks/mvg_generator.hpp"

#include <gtest/gtest.h>

#include "geometry/lie.hpp"
#include "testing_utilities/constants.hpp"
#include "types/eigen_types.hpp"

using namespace reprojection;

// NOTE(Jack): All points for every frame project successfully. If not they should get masked out, but the
// test data is engineered such that none get masked out which is why we can assert that .rows() = 25 for all
// frames.
// But don't forget that there might be an implementation error because when we set the view point and
// sphere origin as {0,0,0} we get poses that do not make sense!
TEST(TestingMocksMvgGenerator, TestGenerateBatch) {
    CameraCalibrationData const batch{testing_mocks::GenerateMvgData(
        100, CameraModel::Pinhole, testing_utilities::pinhole_intrinsics, testing_utilities::image_bounds, false)};

    uint64_t gt_timestamp_ns{0};
    for (auto const& [timestamp_ns, frame_i] : batch.frames) {
        EXPECT_EQ(frame_i.extracted_target.bundle.pixels.rows(), 25);
        EXPECT_EQ(frame_i.extracted_target.bundle.points.rows(), 25);
        EXPECT_EQ(timestamp_ns, gt_timestamp_ns);

        gt_timestamp_ns += 1000000;
    }

    // The actual last timestamps in batch.frames is 99000000 but the += to the ground truth value happens one more time
    // after the last data is read which gives us this timestamp large by one time increment.
    EXPECT_EQ(gt_timestamp_ns, 99000000 + 1000000);
}

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

TEST(TestingMocksNoiseGeneration, TestAddGaussianNoise) {
    auto const identity{Isometry3d::Identity()};

    double const sigma_translation{0.3};
    double const sigma_rotation{0.15};

    int trial_count{1000};
    MatrixX3d translations{MatrixX3d(trial_count, 3)};
    MatrixX3d rotations_se3{MatrixX3d(trial_count, 3)};
    for (int i{0}; i < trial_count; ++i) {
        Isometry3d const perturbed{testing_mocks::AddGaussianNoise(sigma_translation, sigma_rotation, identity)};

        translations.row(i) = perturbed.translation();
        rotations_se3.row(i) = geometry::Log<double>(perturbed.rotation());
    }

    EXPECT_NEAR(translations.mean(), 0.0, 2e-2);
    EXPECT_NEAR(rotations_se3.mean(), 0.0, 1e-2);

    // TODO(Jack): We should add a function to for this calculate and assert mean/covariance logic. It is used here
    //  twice and once in TestGaussianNoise
    MatrixXd const translations_centered{translations.array() - translations.mean()};
    double const translations_sigma{std::sqrt((translations_centered.array() * translations_centered.array()).mean())};
    EXPECT_NEAR(translations_sigma, sigma_translation, 2e-2);

    MatrixXd const rotations_se3_centered{rotations_se3.array() - rotations_se3.mean()};
    double const rotations_se3_sigma{
        std::sqrt((rotations_se3_centered.array() * rotations_se3_centered.array()).mean())};
    EXPECT_NEAR(rotations_se3_sigma, sigma_rotation, 1e-2);
}