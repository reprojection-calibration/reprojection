#include "feature_extraction/target_extraction.hpp"

#include <gtest/gtest.h>

#include "target_extractors.hpp"
#include "test_fixture_yaml_config.hpp"

using namespace reprojection::feature_extraction;

// TODO(Jack): Here we program that detector to not detect the target, maybe if we had better test fixtures we could use
// one here and actually get successful detections.
auto const empty_image{cv::Mat::zeros(cv::Size(100, 100), CV_8UC1)};

TEST_F(YamlConfigTestFixture, TestCreateTargetExtractorCheckerboard) {
    std::unique_ptr<TargetExtractor> const extractor{CreateTargetExtractor(checkerboard_)};

    ASSERT_TRUE(dynamic_cast<CheckerboardExtractor*>(extractor.get()));
    EXPECT_EQ(extractor->Extract(empty_image),
              std::nullopt);  // Failure - mainly added this part of the test to get 100% code coverage, these extract
                              // methods are really tested in each detector's implementation much better.
}

TEST_F(YamlConfigTestFixture, TestCreateTargetExtractorCircleGrid) {
    std::unique_ptr<TargetExtractor> const extractor{CreateTargetExtractor(circle_grid_)};

    ASSERT_TRUE(dynamic_cast<CircleGridExtractor*>(extractor.get()));
    EXPECT_EQ(extractor->Extract(empty_image), std::nullopt);
}

TEST_F(YamlConfigTestFixture, TestCreateTargetExtractorAprilGrid3) {
    std::unique_ptr<TargetExtractor> const extractor{CreateTargetExtractor(april_grid3_)};

    ASSERT_TRUE(dynamic_cast<AprilGrid3Extractor*>(extractor.get()));
    EXPECT_EQ(extractor->Extract(empty_image), std::nullopt);
}
