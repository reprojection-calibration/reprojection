#include "config/config_parse.hpp"

#include <gtest/gtest.h>

using namespace reprojection;

TEST(ConfigParsingHelpers, TestConfigParse) {
    static constexpr std::string_view full_table{R"(
        [application]
        show_extraction = true
        threads = 10

        [camera]
        sensor_name = "/cam0/image_raw"
        camera_model = "double_sphere"

        [imu]
        sensor_name = "/imu0"

        [target]
        type = "checkerboard"
        pattern_size = [3,4]
        unit_dimension = 0.5


        [target.circle_grid]
        asymmetric = true
    )"};
    toml::table const full_config{toml::parse(full_table)};

    auto const result = config::Config::Parse(full_config);

    EXPECT_EQ(result.application.show_extraction, true);
    EXPECT_EQ(result.application.threads, 10);
    EXPECT_EQ(result.camera.sensor_name, "/cam0/image_raw");
    EXPECT_EQ(result.camera.camera_model, CameraModel::DoubleSphere);
    ASSERT_TRUE(result.imu.has_value());
    EXPECT_EQ(result.imu->sensor_name, "/imu0");
    EXPECT_EQ(result.target.target_type, TargetType::Checkerboard);
    EXPECT_EQ(result.target.size[0], 3);
    EXPECT_EQ(result.target.size[1], 4);
    EXPECT_EQ(result.target.unit_dimension, 0.5);
    EXPECT_EQ(result.target.asymmetric, true);
}