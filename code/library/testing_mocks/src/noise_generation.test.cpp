#include "noise_generation.hpp"

#include <gtest/gtest.h>

using namespace reprojection;

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
