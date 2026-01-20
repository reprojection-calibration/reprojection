#include "testing_mocks/mvg_generator.hpp"

#include <gtest/gtest.h>

#include "geometry/lie.hpp"
#include "types/eigen_types.hpp"

using namespace reprojection;

TEST(TestingMocksMvgGenerator, TestGenerateBatch) {
    testing_mocks::MvgGenerator const generator{
        testing_mocks::MvgGenerator(CameraModel::Pinhole, Array4d{600, 600, 360, 240}, {0, 720, 0, 480}, false)};

    // NOTE(Jack): All points for every frame project successfully. If not they should get masked out, but the
    // test data is engineered such that none get masked out. But don't forget that there might be an implementation
    // error because when we set the view point and sphere origin as {0,0,0} we get poses that do not make sense!
    CameraCalibrationData const batch{generator.GenerateBatch(100)};
    for (auto const& [_, frame_i] : batch.frames) {
        EXPECT_EQ(frame_i.extracted_target.bundle.pixels.rows(), 25);
        EXPECT_EQ(frame_i.extracted_target.bundle.points.rows(), 25);
    }
}

TEST(TestingMocksMvgGenerator, TestProject) {
    MatrixX3d const points_w{{0.00, 0.00, 5.00},   {1.00, 1.00, 5.00},   {-1.00, -1.00, 5.00},
                             {2.00, -1.00, 10.00}, {-2.00, 1.00, 10.00}, {0.50, -0.50, 7.00}};
    MatrixX2d const gt_pixels{{360.00, 240.00}, {480.00, 360.00}, {240.00, 120.00},
                              {480.00, 180.00}, {240.00, 300.00}, {402.857, 197.144}};

    auto const camera{std::unique_ptr<projection_functions::Camera>(
        new projection_functions::PinholeCamera({600, 600, 360, 240}, {0, 720, 0, 480}))};
    Isometry3d const tf_co_w{Isometry3d::Identity()};

    auto const [pixels, mask]{testing_mocks::MvgGenerator::Project(points_w, camera, tf_co_w)};
    ASSERT_TRUE(mask.all());
    EXPECT_TRUE(pixels.isApprox(gt_pixels, 1e-3));
}

TEST(TestingMocksMvgGenerator, TestProjectMasking) {
    // Given the tf_co_w transform set below, the last point here will be behind the camera and should be masked out!
    MatrixX3d const points_w{{0.00, 0.00, 10.00},  //
                             {1.00, 1.00, 10.00},
                             {-1.00, -1.00, 5.00}};
    Array3<bool> const gt_mask{true, true, false};

    auto const camera{std::unique_ptr<projection_functions::Camera>(
        new projection_functions::PinholeCamera({600, 600, 360, 240}, {0, 720, 0, 480}))};
    Isometry3d tf_co_w{Isometry3d::Identity()};
    tf_co_w.translation().z() = -6.0;

    auto const [pixels, mask]{testing_mocks::MvgGenerator::Project(points_w, camera, tf_co_w)};
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