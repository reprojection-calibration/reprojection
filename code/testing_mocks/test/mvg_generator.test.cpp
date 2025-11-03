#include "testing_mocks/mvg_generator.hpp"

#include <gtest/gtest.h>

#include "types/eigen_types.hpp"

using namespace reprojection;

TEST(TestingMocksMvgGenerator, TestGenerateNoError) {
    testing_mocks::MvgGenerator const generator{testing_mocks::MvgGenerator(
        std::unique_ptr<projection_functions::Camera>(new projection_functions::PinholeCamera({600, 600, 360, 240})),
        false)};

    EXPECT_NO_FATAL_FAILURE(generator.GenerateBatchFrames(100));
}

TEST(TestingMocksMvgGenerator, TestProject) {
    Eigen::MatrixX3d const points_w{{0.00, 0.00, 5.00},   {1.00, 1.00, 5.00},   {-1.00, -1.00, 5.00},
                                    {2.00, -1.00, 10.00}, {-2.00, 1.00, 10.00}, {0.50, -0.50, 7.00}};

    auto const camera{
        std::unique_ptr<projection_functions::Camera>(new projection_functions::PinholeCamera({600, 600, 360, 240}))};
    Eigen::Isometry3d const tf_co_w{Eigen::Isometry3d::Identity()};
    Eigen::MatrixX2d const pixels{testing_mocks::MvgGenerator::Project(points_w, camera, tf_co_w)};

    Eigen::MatrixX2d const test_pixels{{360.00, 240.00}, {480.00, 360.00}, {240.00, 120.00},
                                       {480.00, 180.00}, {240.00, 300.00}, {402.857, 197.144}};
    ASSERT_TRUE(pixels.isApprox(test_pixels, 1e-3));
}