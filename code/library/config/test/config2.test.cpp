#include "config/config2.hpp"

#include <gtest/gtest.h>

#include "testing_utilities/temporary_file.hpp"

using namespace reprojection;
using TemporaryFile = testing_utilities::TemporaryFile;

TEST(ConfigConfig2, TestConfigLoad) {
    static constexpr std::string_view table_content{R"(
        [application]
        show_extraction = true
        threads = 5

        [camera]
        sensor_name = "/cam0/image_raw"
        camera_model = "double_sphere"
    )"};
    TemporaryFile const config_file{".toml", table_content};

    auto result{config::Config::Load(config_file.Path())};

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->app.show_extraction, true);
    EXPECT_EQ(result->app.threads, 5);
    EXPECT_EQ(result->camera.sensor_name, "/cam0/image_raw");
    EXPECT_EQ(result->camera.camera_model, CameraModel::DoubleSphere);
}

TEST(ConfigConfig2, TestConfigApplicationParse) {
    static constexpr std::string_view empty_table{R"(
    )"};
    toml::table toml{toml::parse(empty_table)};

    // Parsing an empty table gives us the default values.
    auto result{config::Config::Application::Parse(toml)};
    EXPECT_TRUE(std::holds_alternative<config::Config::Application>(result));
    EXPECT_EQ(std::get<config::Config::Application>(result).show_extraction, false);
    // NOTE(Jack): We use greater than or equal here because we do not know how many threads a computer will have.
    EXPECT_GE(std::get<config::Config::Application>(result).threads, 1);

    static constexpr std::string_view full_table{R"(
        show_extraction = true
        threads = 5
    )"};
    toml = toml::parse(full_table);

    // Parsing a full table lets us specify all values.
    result = config::Config::Application::Parse(toml);
    EXPECT_TRUE(std::holds_alternative<config::Config::Application>(result));
    EXPECT_EQ(std::get<config::Config::Application>(result).show_extraction, true);
    EXPECT_EQ(std::get<config::Config::Application>(result).threads, 5);

    static constexpr std::string_view unwanted_keys_table{R"(
        show_extraction = true
        threads = 5

        key1 = "value1"
        key2 = [1, 2]
    )"};
    toml = toml::parse(unwanted_keys_table);

    result = config::Config::Application::Parse(toml);
    EXPECT_TRUE(std::holds_alternative<TomlErrorMsg>(result));
    EXPECT_EQ(std::get<TomlErrorMsg>(result).type, TomlError::UnknownKey);
    EXPECT_EQ(std::get<TomlErrorMsg>(result).msg, "{'key1': 'value1', 'key2': [ 1, 2 ]}");
}

TEST(ConfigConfig2, TestConfigCameraParse) {
    static constexpr std::string_view empty_table{R"(
    )"};
    toml::table toml{toml::parse(empty_table)};

    // Parsing an empty table is an error for us here because the sensor name and camera model do not have sensible
    // default values.
    auto result{config::Config::Camera::Parse(toml)};
    EXPECT_TRUE(std::holds_alternative<TomlErrorMsg>(result));
    EXPECT_EQ(std::get<TomlErrorMsg>(result).type, TomlError::MissingKey);
    EXPECT_EQ(std::get<TomlErrorMsg>(result).msg, "{'sensor_name': 'N/A', 'camera_model': 'N/A'}");

    static constexpr std::string_view full_table{R"(
        sensor_name = "/cam0/image_raw"
        camera_model = "double_sphere"
    )"};
    toml = toml::parse(full_table);

    // Parsing a full table lets us specify all values.
    result = config::Config::Camera::Parse(toml);
    EXPECT_TRUE(std::holds_alternative<config::Config::Camera>(result));
    EXPECT_EQ(std::get<config::Config::Camera>(result).sensor_name, "/cam0/image_raw");
    EXPECT_EQ(std::get<config::Config::Camera>(result).camera_model, CameraModel::DoubleSphere);

    static constexpr std::string_view unwanted_keys_table{R"(
        sensor_name = "/cam0/image_raw"
        camera_model = "double_sphere"

        key1 = "value1"
        key2 = [1, 2]
    )"};
    toml = toml::parse(unwanted_keys_table);

    result = config::Config::Camera::Parse(toml);
    EXPECT_TRUE(std::holds_alternative<TomlErrorMsg>(result));
    EXPECT_EQ(std::get<TomlErrorMsg>(result).type, TomlError::UnknownKey);
    EXPECT_EQ(std::get<TomlErrorMsg>(result).msg, "{'key1': 'value1', 'key2': [ 1, 2 ]}");
}

TEST(ConfigConfig2, TestConfigImuParse) {
    static constexpr std::string_view empty_table{R"(
    )"};
    toml::table toml{toml::parse(empty_table)};

    auto result{config::Config::Imu::Parse(toml)};
    EXPECT_TRUE(std::holds_alternative<TomlErrorMsg>(result));
    EXPECT_EQ(std::get<TomlErrorMsg>(result).type, TomlError::MissingKey);
    EXPECT_EQ(std::get<TomlErrorMsg>(result).msg, "{'sensor_name': 'N/A'}");

    static constexpr std::string_view full_table{R"(
        sensor_name = "/imu0"
    )"};
    toml = toml::parse(full_table);

    result = config::Config::Imu::Parse(toml);
    EXPECT_TRUE(std::holds_alternative<config::Config::Imu>(result));
    EXPECT_EQ(std::get<config::Config::Imu>(result).sensor_name, "/imu0");

    static constexpr std::string_view unwanted_keys_table{R"(
        sensor_name = "/imu0"

        key1 = "value1"
        key2 = [1, 2]
    )"};
    toml = toml::parse(unwanted_keys_table);

    result = config::Config::Imu::Parse(toml);
    EXPECT_TRUE(std::holds_alternative<TomlErrorMsg>(result));
    EXPECT_EQ(std::get<TomlErrorMsg>(result).type, TomlError::UnknownKey);
    EXPECT_EQ(std::get<TomlErrorMsg>(result).msg, "{'key1': 'value1', 'key2': [ 1, 2 ]}");
}

TEST(ConfigConfig2, TestConfigTargetParse) {
    static constexpr std::string_view empty_table{R"(
    )"};
    toml::table toml{toml::parse(empty_table)};

    auto result{config::Config::Target::Parse(toml)};
    EXPECT_TRUE(std::holds_alternative<TomlErrorMsg>(result));
    EXPECT_EQ(std::get<TomlErrorMsg>(result).type, TomlError::MissingKey);
    EXPECT_EQ(std::get<TomlErrorMsg>(result).msg, "{'type': 'N/A', 'pattern_size': 'N/A'}");

    static constexpr std::string_view required_table{R"(
        type = "checkerboard"
        pattern_size = [3,4]
    )"};
    toml = toml::parse(required_table);

    result = config::Config::Target::Parse(toml);
    EXPECT_TRUE(std::holds_alternative<config::Config::Target>(result));
    EXPECT_EQ(std::get<config::Config::Target>(result).target_type, TargetType::Checkerboard);
    EXPECT_EQ(std::get<config::Config::Target>(result).size[0], 3);
    EXPECT_EQ(std::get<config::Config::Target>(result).size[1], 4);
    EXPECT_EQ(std::get<config::Config::Target>(result).unit_dimension, 1);
    EXPECT_EQ(std::get<config::Config::Target>(result).asymmetric, false);

    static constexpr std::string_view full_table{R"(
        type = "circle_grid"
        pattern_size = [3,4]
        unit_dimension = 1.1

        [circle_grid]
        asymmetric = true
    )"};
    toml = toml::parse(full_table);

    result = config::Config::Target::Parse(toml);
    EXPECT_TRUE(std::holds_alternative<config::Config::Target>(result));
    EXPECT_EQ(std::get<config::Config::Target>(result).target_type, TargetType::CircleGrid);
    EXPECT_EQ(std::get<config::Config::Target>(result).unit_dimension, 1.1);
    EXPECT_EQ(std::get<config::Config::Target>(result).asymmetric, true);

    static constexpr std::string_view unwanted_keys_table{R"(
        type = "checkerboard"
        pattern_size = [3,4]

        key1 = "value1"
        key2 = [1, 2]
    )"};
    toml = toml::parse(unwanted_keys_table);

    result = config::Config::Target::Parse(toml);
    EXPECT_TRUE(std::holds_alternative<TomlErrorMsg>(result));
    EXPECT_EQ(std::get<TomlErrorMsg>(result).type, TomlError::UnknownKey);
    EXPECT_EQ(std::get<TomlErrorMsg>(result).msg, "{'key1': 'value1', 'key2': [ 1, 2 ]}");

    static constexpr std::string_view unwanted_keys_table_2{R"(
        type = "checkerboard"
        pattern_size = [3,4]

        [circle_grid]
        asymmetric = true

        key11 = "value1"
        key22 = [1, 2]
    )"};
    toml = toml::parse(unwanted_keys_table_2);

    result = config::Config::Target::Parse(toml);
    EXPECT_TRUE(std::holds_alternative<TomlErrorMsg>(result));
    EXPECT_EQ(std::get<TomlErrorMsg>(result).type, TomlError::UnknownKey);
    EXPECT_EQ(std::get<TomlErrorMsg>(result).msg, "{'key11': 'value1', 'key22': [ 1, 2 ]}");
}