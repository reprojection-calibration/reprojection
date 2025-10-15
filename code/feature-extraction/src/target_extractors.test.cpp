#include "target_extractors.hpp"

#include <gtest/gtest.h>

#include "target_generators.hpp"
#include "test_fixture_april_tag.hpp"
#include "utilities.hpp"  // REMOVE

using namespace reprojection_calibration::feature_extraction;

TEST(TargetExtractors, TestCheckerboardExtractor) {
    cv::Size const pattern_size{4, 3};  // (width, height) == (cols, rows)
    int const square_size_pixels{50};
    cv::Mat const image{GenerateCheckerboard(pattern_size, square_size_pixels)};

    double const unit_dimension{0.5};
    auto const extractor{CheckerboardExtractor{pattern_size, unit_dimension}};

    std::optional<FeatureFrame> const target{extractor.Extract(image)};
    ASSERT_TRUE(target.has_value());

    Eigen::MatrixX2d const& pixels{target->pixels};
    EXPECT_EQ(pixels.rows(), pattern_size.height * pattern_size.width);
    EXPECT_TRUE(pixels.row(0).isApprox(Eigen::Vector2d{100, 100}.transpose(), 1e-6));   // First pixel - heuristic
    EXPECT_TRUE(pixels.row(11).isApprox(Eigen::Vector2d{250, 200}.transpose(), 1e-6));  // Last pixel - heuristic

    Eigen::MatrixX3d const& points{target->points};
    EXPECT_EQ(points.rows(), pattern_size.height * pattern_size.width);
    EXPECT_TRUE(points.row(0).isApprox(Eigen::Vector3d{0, 0, 0}.transpose()));     // First pixel - heuristic
    EXPECT_TRUE(points.row(11).isApprox(Eigen::Vector3d{1, 1.5, 0}.transpose()));  // Last pixel - heuristic

    Eigen::ArrayX2i const& indices{target->indices};
    EXPECT_EQ(indices.rows(), pattern_size.width * pattern_size.height);
    EXPECT_TRUE(indices.row(0).isApprox(Eigen::Vector2i{0, 0}.transpose()));   // First index - heuristic
    EXPECT_TRUE(indices.row(11).isApprox(Eigen::Vector2i{2, 3}.transpose()));  // Last index - heuristic
}

TEST(TargetExtractors, TestCircleGridExtractor) {
    cv::Size const pattern_size{4, 3};
    int const circle_radius_pixels{25};
    int const circle_spacing_pixels{20};  // Between circle edges
    bool const asymmetric{false};
    cv::Mat const image{GenerateCircleGrid(pattern_size, circle_radius_pixels, circle_spacing_pixels, asymmetric)};

    double const unit_dimension{0.5};
    auto const extractor{CircleGridExtractor{pattern_size, unit_dimension, asymmetric}};

    std::optional<FeatureFrame> const target{extractor.Extract(image)};
    ASSERT_TRUE(target.has_value());

    Eigen::MatrixX2d const& pixels{target->pixels};
    EXPECT_EQ(pixels.rows(), pattern_size.width * pattern_size.height);
    EXPECT_TRUE(pixels.row(0).isApprox(Eigen::Vector2d{265, 195}.transpose(), 1e-6));
    EXPECT_TRUE(pixels.row(11).isApprox(Eigen::Vector2d{55, 55}.transpose(), 1e-6));

    Eigen::MatrixX3d const& points{target->points};
    EXPECT_EQ(points.rows(), pattern_size.height * pattern_size.width);
    EXPECT_TRUE(points.row(0).isApprox(Eigen::Vector3d{0, 0, 0}.transpose()));
    EXPECT_TRUE(points.row(11).isApprox(Eigen::Vector3d{1, 1.5, 0}.transpose()));

    Eigen::ArrayX2i const& indices{target->indices};
    EXPECT_EQ(indices.rows(), pattern_size.height * pattern_size.width);
    EXPECT_TRUE(indices.row(0).isApprox(Eigen::Vector2i{0, 0}.transpose()));
    EXPECT_TRUE(indices.row(11).isApprox(Eigen::Vector2i{2, 3}.transpose()));
}

TEST(TargetExtractors, TestCircleGridExtractorAsymmetric) {
    // Refactor to use cv::Size
    // WARN(Jack): Must be even (rows)! See comment below.
    // WARN(Jack): Must be an odd number (cols) to prevent 180 degree rotation symmetry!
    // https://answers.opencv.org/question/96561/calibration-with-findcirclesgrid-trouble-with-pattern-widthheight/
    cv::Size const pattern_size{7, 6};
    int const circle_radius_pixels{25};
    int const circle_spacing_pixels{20};
    bool const asymmetric{true};
    cv::Mat image{GenerateCircleGrid(pattern_size, circle_radius_pixels, circle_spacing_pixels, asymmetric)};

    double const unit_dimension{0.5};
    auto const extractor{CircleGridExtractor{pattern_size, unit_dimension, asymmetric}};

    std::optional<FeatureFrame> const target{extractor.Extract(image)};
    ASSERT_TRUE(target.has_value());

    Eigen::MatrixX2d const& pixels{target->pixels};
    EXPECT_EQ(pixels.rows(),
              (pattern_size.width * pattern_size.height) / 2);  // NOTE(Jack): Divide by two due to asymmetry!
    EXPECT_TRUE(pixels.row(0).isApprox(Eigen::Vector2d{475, 55}.transpose(), 1e-6));
    EXPECT_TRUE(pixels.row(20).isApprox(Eigen::Vector2d{55, 335}.transpose(), 1e-6));

    Eigen::MatrixX3d const& points{target->points};
    EXPECT_EQ(points.rows(), (pattern_size.width * pattern_size.height) / 2);
    EXPECT_TRUE(points.row(0).isApprox(Eigen::Vector3d{0, 0, 0}.transpose()));
    EXPECT_TRUE(points.row(20).isApprox(Eigen::Vector3d{3, 2, 0}.transpose()));

    Eigen::ArrayX2i const& indices{target->indices};
    EXPECT_EQ(indices.rows(), (pattern_size.width * pattern_size.height) / 2);
    EXPECT_TRUE(indices.row(0).isApprox(Eigen::Vector2i{0, 0}.transpose()));
    EXPECT_TRUE(indices.row(20).isApprox(Eigen::Vector2i{6, 4}.transpose()));
}

TEST_F(AprilTagTestFixture, TestAprilGrid3Extractor) {
    cv::Mat const april_tag{AprilBoard3Generation::GenerateTag(bit_size_pixel_, code_matrix_0_)};

    cv::Size const pattern_size{4, 3};
    double const unit_dimension{0.5};
    auto const extractor{AprilGrid3Extractor{pattern_size, unit_dimension}};

    std::optional<FeatureFrame> const target{extractor.Extract(april_tag)};
    ASSERT_TRUE(target.has_value());

    Eigen::MatrixX2d const& pixels{target->pixels};
    EXPECT_EQ(pixels.rows(), 4);  // One tag
    Eigen::Matrix<double, 4, 2> const gt_pixels{{19.685731887817383, 19.685731887817383},
                                                {119.27910614013672, 19.819416046142578},
                                                {19.819417953491211, 119.27910614013672},
                                                {119.13014984130859, 119.13014984130859}};
    EXPECT_TRUE(pixels.isApprox(gt_pixels, 1e-6));

    Eigen::MatrixX3d const& points{target->points};
    Eigen::Matrix<double, 4, 3> const gt_points{{0, 0, 0}, {0.5, 0, 0}, {0, 0.5, 0}, {0.5, 0.5, 0}};
    EXPECT_TRUE(points.isApprox(gt_points));

    Eigen::ArrayX2i const& indices{target->indices};
    Eigen::Array<int, 4, 2> const gt_indices{{0, 0}, {0, 1}, {1, 0}, {1, 1}};
    EXPECT_TRUE(indices.isApprox(gt_indices));
}

TEST_F(AprilTagTestFixture, TestAprilGrid3VisibleGeometry) {
    cv::Size const pattern_size{3, 2};

    // Make a simulated set of detections from a 3x2 AprilGrid3 - we only need the ID here
    std::vector<AprilTagDetection> detections;
    for (int i{0}; i < pattern_size.width * pattern_size.height; ++i) {
        AprilTagDetection detection_i;
        detection_i.id = i;
        detections.push_back(detection_i);
    }

    Eigen::ArrayXi const mask1{AprilGrid3Extractor::VisibleGeometry(pattern_size, detections)};
    EXPECT_EQ(mask1.rows(), 4 * pattern_size.width * pattern_size.height);
    EXPECT_TRUE(mask1.topRows(4).isApprox(Eigen::Array<int, 4, 1>{0, 1, 6, 7}));
    EXPECT_TRUE(mask1.bottomRows(4).isApprox(Eigen::Array<int, 4, 1>{16, 17, 22, 23}));

    // Now remove the first tag detection and see that it still works
    detections.erase(std::begin(detections));

    Eigen::ArrayXi const mask2{AprilGrid3Extractor::VisibleGeometry(pattern_size, detections)};
    EXPECT_EQ(mask2.rows(), 4 * (pattern_size.width * pattern_size.height - 1));  // Four fewer elements
    EXPECT_TRUE(mask2.topRows(4).isApprox(Eigen::Array<int, 4, 1>{2, 3, 8, 9}));  // Moved one tag over along the row
    EXPECT_TRUE(mask2.bottomRows(4).isApprox(Eigen::Array<int, 4, 1>{16, 17, 22, 23}));  // Same as before removal
}

TEST_F(AprilTagTestFixture, TestAprilGrid3CornerPositions) {
    // Should be even because aprilgrids always have even points in each direction because it is always a multiple of
    // two of the board's tag rows/columns
    Eigen::ArrayX2i const grid{GenerateGridIndices(6, 8)};
    Eigen::MatrixX3d const points{AprilGrid3Extractor::CornerPositions(grid, 0.5)};

    EXPECT_EQ(points.rows(), grid.rows());
    EXPECT_TRUE(points.row(0).isApprox(Eigen::Vector3d{0, 0, 0}.transpose()));
    EXPECT_TRUE(points.row(47).isApprox(Eigen::Vector3d{2.6, 1.9, 0}.transpose()));
};