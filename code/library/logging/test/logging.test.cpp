#include "logging/logging.hpp"

#include <gtest/gtest.h>

using namespace reprojection;

TEST(LoggingLogging, TestToOneLineJson) {
    // TODO(Jack): Copy and pasted in many places!
    static constexpr std::string_view minimum_config{R"(
        [sensor]
        camera_name = "/cam0/image_raw"
        camera_model = "double_sphere"

        [target]
        pattern_size = [3,4]
        type = "circle_grid"
    )"};

    toml::table const toml{toml::parse(minimum_config)};

    std::string const result{logging::ToOneLineJson(toml)};

    EXPECT_EQ(result,
              "{'sensor': {'camera_model': 'double_sphere', 'camera_name': '/cam0/image_raw'}, 'target': "
              "{'pattern_size': [3, 4], 'type': 'circle_grid'}}");
}