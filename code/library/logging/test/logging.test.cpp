#include "logging/logging.hpp"

#include <gtest/gtest.h>

// cppcheck-suppress missingInclude
#include "testing_utilities/generated/minimum_config.hpp"

using namespace reprojection;

TEST(LoggingLogging, TestToOneLineJson) {
    toml::table const toml{toml::parse(testing_utilities::minimum_config)};

    std::string const result{logging::ToOneLineJson(toml)};

    EXPECT_EQ(result,
              "{'application': {'show_extraction': false}, 'camera': {'camera_model': 'double_sphere', 'sensor_name': "
              "'/cam0/image_raw'}, 'imu': {'sensor_name': '/imu0'}, 'target': {'pattern_size': [5, 4], 'type': "
              "'aprilgrid3'}}");
}