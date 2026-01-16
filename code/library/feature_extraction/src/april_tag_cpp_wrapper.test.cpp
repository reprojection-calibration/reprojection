#include "april_tag_cpp_wrapper.hpp"

#include <gtest/gtest.h>

#include "test_fixture_april_tag.hpp"
#include "types/eigen_types.hpp"

using namespace reprojection;
using namespace reprojection::feature_extraction;

TEST_F(AprilTagTestFixture, TestAprilTagDetectorDetectAprilBoard) {
    cv::Size const pattern_size{4, 3};
    cv::Mat const april_board{AprilBoard3Generation::GenerateBoard(
        tag_family_handler_.tag_family->nbits, tag_family_handler_.tag_family->codes, bit_size_pixel_, pattern_size)};

    std::vector<AprilTagDetection> const detections{tag_detector_.Detect(april_board)};

    int const num_tags{pattern_size.height * pattern_size.width};
    EXPECT_EQ(std::size(detections), num_tags);
    for (int i = 0; i < num_tags; i++) {
        EXPECT_EQ(detections[i].id, i);  // Tag IDs should always be generated in order as [0, num_tags)
    }
}

TEST_F(AprilTagTestFixture, TestAprilTagDetectorDetectAprilTag) {
    cv::Mat const april_tag{AprilBoard3Generation::GenerateTag(bit_size_pixel_, code_matrix_0_)};

    std::vector<AprilTagDetection> const detections{tag_detector_.Detect(april_tag)};
    EXPECT_EQ(std::size(detections), 1);

    AprilTagDetection const detection{detections[0]};
    EXPECT_EQ(detection.id, 0);

    // Center point
    EXPECT_TRUE(detection.c.isApproxToConstant(71));

    // The four corner pixels in the apriltag native order.
    Matrix42d gt_pixels{{30, 112}, {112, 112}, {112, 30}, {30, 30}};
    EXPECT_TRUE(detection.p.isApprox(gt_pixels));
}
