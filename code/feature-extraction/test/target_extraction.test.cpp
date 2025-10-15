#include "feature_extraction/target_extraction.hpp"

#include <gtest/gtest.h>

#include "target_extractors.hpp"
#include "test_fixture_yaml_config.hpp"

using namespace reprojection_calibration::feature_extraction;

TEST_F(YamlConfigTestFixture, TestCreateTargetExtractorCheckerboard) {
    std::unique_ptr<TargetExtractor> const extractor{CreateTargetExtractor(checkerboard_)};

    EXPECT_TRUE(dynamic_cast<CheckerboardExtractor*>(extractor.get()));
}

TEST_F(YamlConfigTestFixture, TestCreateTargetExtractorCircleGrid) {
    std::unique_ptr<TargetExtractor> const extractor{CreateTargetExtractor(circle_grid_)};

    EXPECT_TRUE(dynamic_cast<CircleGridExtractor*>(extractor.get()));
}

TEST_F(YamlConfigTestFixture, TestCreateTargetExtractorAprilGrid3) {
    std::unique_ptr<TargetExtractor> const extractor{CreateTargetExtractor(april_grid3_)};

    EXPECT_TRUE(dynamic_cast<AprilGrid3Extractor*>(extractor.get()));
}
