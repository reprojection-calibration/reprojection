#include "noise_generation.hpp"

#include <gtest/gtest.h>

#include "geometry/lie.hpp"

using namespace reprojection;

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
        rotations_se3.row(i) = geometry::Log(perturbed.rotation());
    }

    //
    EXPECT_NEAR(translations.mean(), 0.0, 2e-2);
    EXPECT_NEAR(rotations_se3.mean(), 0.0, 1e-2);

    // TODO(Jack): We should add a function to for this calcualte and assert mean/covariance logic. It is used here
    // twice and once in TestGaussianNoise
    MatrixXd const translations_centered{translations.array() - translations.mean()};
    double const translations_sigma{std::sqrt((translations_centered.array() * translations_centered.array()).mean())};
    EXPECT_NEAR(translations_sigma, sigma_translation, 1e-2);

    MatrixXd const rotations_se3_centered{rotations_se3.array() - rotations_se3.mean()};
    double const rotations_se3_sigma{
        std::sqrt((rotations_se3_centered.array() * rotations_se3_centered.array()).mean())};
    EXPECT_NEAR(rotations_se3_sigma, sigma_rotation, 1e-2);
}

TEST(TestingMocksNoiseGeneration, TestGaussianNoise) {
    double const mean{1.75};
    double const sigma{0.1};
    int const rows{10000};
    int const cols{3};

    // NOTE(Jack): Because the generated noise is totally independent we do not need to worry about do anything based on
    // the rows of columns. We can simply calculate the mean and covariance using every element directly.
    MatrixX3d const a{testing_mocks::GaussianNoise(mean, sigma, rows, cols)};
    EXPECT_NEAR(a.mean(), mean, 1e-2);

    MatrixXd const a_centered{a.array() - a.mean()};
    double const a_sigma{std::sqrt((a_centered.array() * a_centered.array()).mean())};
    EXPECT_NEAR(a_sigma, sigma, 1e-2);
}
