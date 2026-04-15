#include "feature_extraction/target_extraction.hpp"

#include <gtest/gtest.h>

#include <string_view>

#include "target_extractors.hpp"

using namespace reprojection;
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
    pattern_size = [3, 4]
    type = "circle_grid"
    unit_dimension = 1.0

    [circle_grid]
    asymmetric = true
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

TEST(FeatureExtractionTargetExtraction, TestCreateTargetExtractorErrorHandling) {
    static constexpr std::string_view empty_config_file{R"()"sv};
    toml::table const config{toml::parse(empty_config_file)};

    EXPECT_THROW(CreateTargetExtractor(config), std::runtime_error);
}

TEST(FeatureExtractionTargetExtraction, TestDrawTarget) {
    ExtractedTarget const target{{MatrixX2d{{1, 1}, {50, 50}}, MatrixX3d::Random(2, 3)}, ArrayX2i::Random(2, 2)};
    cv::Mat const img{cv::Mat::zeros(cv::Size(100, 100), CV_8UC3)};

    DrawTarget(target, img);

    // Check that at least the two target pixels are colored bright green.
    EXPECT_TRUE(img.at<cv::Vec3b>(1, 1) == cv::Vec3b(0, 255, 0));
    EXPECT_TRUE(img.at<cv::Vec3b>(50, 50) == cv::Vec3b(0, 255, 0));
}