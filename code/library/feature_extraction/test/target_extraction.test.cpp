#include "feature_extraction/target_extraction.hpp"

#include <gtest/gtest.h>

#include <string_view>

#include "target_extractors.hpp"

using namespace reprojection::feature_extraction;
using namespace std::string_view_literals;

// TODO(Jack): Here we program that detector to not detect the target, maybe if we had better test fixtures we could use
// one here and actually get successful detections.
auto const empty_image{cv::Mat::zeros(cv::Size(100, 100), CV_8UC1)};

TEST(FeatureExtractionTargetExtraction, TestCreateTargetExtractorCheckerboard) {
    static constexpr std::string_view config_file{R"(
        pattern_size = [3,4]
        type = "checkerboard"
    )"sv};
    toml::table const config{toml::parse(config_file)};

    std::unique_ptr<TargetExtractor> const extractor{CreateTargetExtractor(config)};

    ASSERT_TRUE(dynamic_cast<CheckerboardExtractor*>(extractor.get()));
    EXPECT_EQ(extractor->Extract(empty_image),
              std::nullopt);  // Failure - mainly added this part of the test to get 100% code coverage, these extract
                              // methods are really tested in each detector's implementation much better.
}

TEST(FeatureExtractionTargetExtraction, TestCreateTargetExtractorCircleGrid) {
    static constexpr std::string_view config_file{R"(
        pattern_size = [3,4]
        type = "circle_grid"
    )"sv};
    toml::table const config{toml::parse(config_file)};

    std::unique_ptr<TargetExtractor> const extractor{CreateTargetExtractor(config)};

    ASSERT_TRUE(dynamic_cast<CircleGridExtractor*>(extractor.get()));
    EXPECT_EQ(extractor->Extract(empty_image), std::nullopt);
}

TEST(FeatureExtractionTargetExtraction, TestCreateTargetExtractorAprilGrid3) {
    static constexpr std::string_view config_file{R"(
        pattern_size = [3,4]
        type = "april_grid3"
    )"sv};
    toml::table const config{toml::parse(config_file)};

    std::unique_ptr<TargetExtractor> const extractor{CreateTargetExtractor(config)};

    ASSERT_TRUE(dynamic_cast<AprilGrid3Extractor*>(extractor.get()));
    EXPECT_EQ(extractor->Extract(empty_image), std::nullopt);
}