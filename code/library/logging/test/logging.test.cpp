#include "logging/logging.hpp"

#include <gtest/gtest.h>

#include "testing_utilities/generated/minimum_config.hpp"

using namespace reprojection;

TEST(LoggingLogging, TestToOneLineJson) {
    toml::table const toml{toml::parse(testing_utilities::minimum_config)};

    std::string const result{logging::ToOneLineJson(toml)};

    EXPECT_EQ(result,
              "{'application': {'show_extraction': false}, 'sensor': {'camera_model': 'double_sphere', 'camera_name': "
              "'/cam0/image_raw'}, 'target': {'pattern_size': [3, 4], 'type': 'aprilgrid3'}}");
}