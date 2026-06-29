#include "config/config_parse.hpp"

#include <gtest/gtest.h>

using namespace reprojection;

TEST(ConfigParsingHelpers, TestConfigParseFull) {
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

TEST(ConfigParsingHelpers, TestConfigParseMinimum) {
    static constexpr std::string_view minimum_table{R"(
        [camera]
        sensor_name = "/cam0/image_raw"
        camera_model = "double_sphere"

        [target]
        type = "checkerboard"
        pattern_size = [3,4]
    )"};
    toml::table const minimum_config{toml::parse(minimum_table)};
    auto const result{config::Config::Parse(minimum_config)};

    EXPECT_EQ(result.application.show_extraction, false);
    EXPECT_GE(result.application.threads, 2);

    EXPECT_EQ(result.camera.sensor_name, "/cam0/image_raw");
    EXPECT_EQ(result.camera.camera_model, CameraModel::DoubleSphere);

    EXPECT_FALSE(result.imu.has_value());

    EXPECT_EQ(result.target.target_type, TargetType::Checkerboard);
    EXPECT_EQ(result.target.size[0], 3);
    EXPECT_EQ(result.target.size[1], 4);
    EXPECT_EQ(result.target.unit_dimension, 1.0);
    EXPECT_EQ(result.target.asymmetric, false);
}

TEST(ConfigParsingHelpers, TestConfigApplicationParse) {
    std::vector<std::string_view> const valid_tables{
        R"()",
        R"(
            show_extraction = true
            threads = 10
        )",
        R"(
            show_extraction = true
        )",
        R"(
            threads = 10
        )",
    };

    for (auto const& valid_table : valid_tables) {
        toml::table const config{toml::parse(valid_table)};

        EXPECT_NO_THROW(config::Config::Application::Parse(config));
    }

    std::vector<std::string_view> const invalid_tables{
        R"(
            [application]
        )",
        R"(
            show_extraction = "wrong_type"
        )",
        R"(
            threads = "wrong_type"
        )",
        R"(
            unexpected_key = "value1"
        )",
    };

    for (auto const& invalid_table : invalid_tables) {
        toml::table const config{toml::parse(invalid_table)};

        EXPECT_THROW(config::Config::Application::Parse(config), std::runtime_error);
    }
}

TEST(ConfigParsingHelpers, TestConfigCameraParse) {
    std::vector<std::string_view> const valid_tables{
        R"(
            sensor_name = "/cam0/image_raw"
            camera_model = "double_sphere"
        )",
    };

    for (auto const& valid_table : valid_tables) {
        toml::table const config{toml::parse(valid_table)};

        EXPECT_NO_THROW(config::Config::Camera::Parse(config));
    }

    std::vector<std::string_view> const invalid_tables{
        R"()",
        R"(
            [camera]
        )",
        R"(
            sensor_name = "/cam0/image_raw"
        )",
        R"(
            camera_model = "double_sphere"
        )",
        R"(
            sensor_name = 1.2
            camera_model = 2.1
        )",
    };

    for (auto const& invalid_table : invalid_tables) {
        toml::table const config{toml::parse(invalid_table)};

        EXPECT_THROW(config::Config::Camera::Parse(config), std::runtime_error);
    }
}
