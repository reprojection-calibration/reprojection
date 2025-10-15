#include "april_tag_cpp_wrapper.hpp"

#include <gtest/gtest.h>

#include "test_fixture_april_tag.hpp"

using namespace reprojection_calibration::feature_extraction;

TEST_F(AprilTagTestFixture, TestAprilTagDetectorDetectAprilBoard) {
    cv::Size const pattern_size{4, 3};
    cv::Mat const april_board{AprilBoard3Generation::GenerateBoard(
        tag_family_handler_.tag_family->nbits, tag_family_handler_.tag_family->codes, bit_size_pixel_, pattern_size)};

    std::vector<AprilTagDetection> const detections{tag_detector_.Detect(april_board)};

    int const num_tags{pattern_size.height * pattern_size.width};
    EXPECT_EQ(std::size(detections), num_tags);
    for (int i = 0; i < num_tags; i++) {
        EXPECT_EQ(detections[i].id, i);  // AprilBoard3 tag IDs will always be generated in order as [0, num_tags)
    }
}

TEST_F(AprilTagTestFixture, TestAprilTagDetectorDetectAprilTag) {
    cv::Mat const april_tag{AprilBoard3Generation::GenerateTag(bit_size_pixel_, code_matrix_0_)};

    std::vector<AprilTagDetection> const detections{tag_detector_.Detect(april_tag)};
    EXPECT_EQ(std::size(detections), 1);

    AprilTagDetection const detection{detections[0]};
    EXPECT_EQ(detection.id, 0);

    // Center point
    EXPECT_FLOAT_EQ(detection.c[0], 71);
    EXPECT_FLOAT_EQ(detection.c[1], 71);

    // TODO(Jack): Replace with groundtruth values in one single matrix and check isApprox()
    // Point zero
    EXPECT_FLOAT_EQ(detection.p(0, 0), 30);
    EXPECT_FLOAT_EQ(detection.p(0, 1), 112);

    // Point one
    EXPECT_FLOAT_EQ(detection.p(1, 0), 112);
    EXPECT_FLOAT_EQ(detection.p(1, 1), 112);

    // Point two
    EXPECT_FLOAT_EQ(detection.p(2, 0), 112);
    EXPECT_FLOAT_EQ(detection.p(2, 1), 30);

    // Point three
    EXPECT_FLOAT_EQ(detection.p(3, 0), 30);
    EXPECT_FLOAT_EQ(detection.p(3, 1), 30);
}
