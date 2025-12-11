#include "feature_extraction/target_extraction.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

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

// NOTE(Jack): This is one of the first tests I really implemented due to the requirment for 100% code coverage. Which
// was a good thing, the errors we throw are an integral part of our interface! Keep in mind that in the project we will
// mainly use errors to kill the program instead of handling them :)
TEST(FeatureExtractionTargetExtraction, TestErrorHandling) {
    YAML::Node config_node;

    // Lacking type
    EXPECT_THROW(CreateTargetExtractor(config_node), std::runtime_error);
    config_node["type"] = "nonexistent_target";

    // Invalid type
    EXPECT_THROW(CreateTargetExtractor(config_node), std::runtime_error);
    config_node["type"] = "april_grid3";

    // Lacking pattern_size
    EXPECT_THROW(CreateTargetExtractor(config_node), std::runtime_error);
    config_node["pattern_size"].push_back(3);
    config_node["pattern_size"].push_back(4);

    // Lacking unit dimension
    EXPECT_THROW(CreateTargetExtractor(config_node), std::runtime_error);
    config_node["unit_dimension"] = 0.1;

    // Now everything is here - no throw
    EXPECT_NO_THROW(CreateTargetExtractor(config_node));

    // Now test circle grid specific logic - requires the asymmetric bool field
    config_node["type"] = "circle_grid";
    EXPECT_THROW(CreateTargetExtractor(config_node), std::runtime_error);
    config_node["circle_grid_options"]["asymmetric"] = false;

    // Now everything for circle grid is here - no throw
    EXPECT_NO_THROW(CreateTargetExtractor(config_node));
}
