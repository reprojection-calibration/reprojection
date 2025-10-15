#include "target_generators.hpp"

#include <gtest/gtest.h>

#include "test_fixture_april_tag.hpp"

using namespace reprojection_calibration::feature_extraction;

TEST(TargetGenerators, TestGenerateCheckboard) {
    cv::Size const pattern_size{4, 3};  // (width, height) == (cols, rows)
    int const square_size_pixels{50};
    cv::Mat const checkerboard_image{GenerateCheckerboard(pattern_size, square_size_pixels)};

    EXPECT_EQ(checkerboard_image.rows, 300);
    EXPECT_EQ(checkerboard_image.cols, 350);
}

TEST(TargetGenerators, TestGenerateCircleGrid) {
    cv::Size const pattern_size{4, 3};
    int const circle_radius_pixels{25};
    int const circle_spacing_pixels{20};  // Distance between circle rims - that means the straight line distance
                                          // between two circles in one row or columns will be (2*radius + spacing)
    bool const asymmetric{false};
    cv::Mat const circlegrid_image{
        GenerateCircleGrid(pattern_size, circle_radius_pixels, circle_spacing_pixels, asymmetric)};

    EXPECT_EQ(circlegrid_image.rows, 250);
    EXPECT_EQ(circlegrid_image.cols, 320);
}

TEST(TargetGenerators, TestGenerateCircleGridAsymmetric) {
    cv::Size const pattern_size{4, 3};
    int const circle_radius_pixels{25};
    int const circle_spacing_pixels{20};
    bool const asymmetric{true};
    cv::Mat const circlegrid_image{
        GenerateCircleGrid(pattern_size, circle_radius_pixels, circle_spacing_pixels, asymmetric)};

    EXPECT_EQ(circlegrid_image.rows, 250);
    EXPECT_EQ(circlegrid_image.cols, 320);
}

TEST_F(AprilTagTestFixture, TestGenerateAprilBoard) {
    cv::Size const pattern_size{4, 3};
    cv::Mat const april_board{AprilBoard3Generation::GenerateBoard(
        tag_family_handler_.tag_family->nbits, tag_family_handler_.tag_family->codes, bit_size_pixel_, pattern_size)};

    EXPECT_EQ(april_board.rows, 420);
    EXPECT_EQ(april_board.cols, 560);
}

TEST_F(AprilTagTestFixture, TestGenerateAprilTag) {
    cv::Mat const april_tag{AprilBoard3Generation::GenerateTag(bit_size_pixel_, code_matrix_0_)};

    EXPECT_EQ(april_tag.rows, 140);
    EXPECT_EQ(april_tag.cols, 140);

    // Test the overrided function matches the original
    cv::Mat const april_tag_1{AprilBoard3Generation::GenerateTag(
        tag_family_handler_.tag_family->nbits, tag_family_handler_.tag_family->codes[0], bit_size_pixel_)};

    EXPECT_TRUE(cv::sum(april_tag != april_tag_1) == cv::Scalar(0));  // I.e. they are the exact same
}

TEST_F(AprilTagTestFixture, TestCalculateCodeMatrix) {
    // Check two properties of the matrix and hope if anything in the implementation breaks these catch it -_- these are
    // heuristics!
    EXPECT_EQ(code_matrix_0_.sum(), 17);
    EXPECT_TRUE(code_matrix_0_.row(5).isApprox(Eigen::Vector<int, 6>{1, 1, 1, 0, 0, 1}.transpose()));
}