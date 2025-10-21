#include "testing_mocks/mvg_generator.hpp"

#include <gtest/gtest.h>

using namespace reprojection::testing_mocks;

TEST(TestingMocks, XXX) {
    MvgGenerator const generator{MvgGenerator(false)};

    int const n{100};
    for (size_t i{0}; i < n; ++i) {
        EXPECT_NO_FATAL_FAILURE(generator.Generate(static_cast<double>(i) / n));
    }
}

TEST(TestingMocks, TestProject) {
    Eigen::MatrixX3d const points_w{{0.00, 0.00, 5.00},   {1.00, 1.00, 5.00},   {-1.00, -1.00, 5.00},
                                    {2.00, -1.00, 10.00}, {-2.00, 1.00, 10.00}, {0.50, -0.50, 7.00}};
    Eigen::Matrix3d const K{{600, 0, 360}, {0, 600, 240}, {0, 0, 1}};

    Eigen::Isometry3d const tf_co_w{Eigen::Isometry3d::Identity()};
    Eigen::MatrixX2d const pixels{MvgGenerator::Project(points_w, K, tf_co_w)};

    Eigen::MatrixX2d const test_pixels{{360.00, 240.00}, {480.00, 360.00}, {240.00, 120.00},
                                       {480.00, 180.00}, {240.00, 300.00}, {402.857, 197.144}};
    ASSERT_TRUE(pixels.isApprox(test_pixels, 1e-3));
}