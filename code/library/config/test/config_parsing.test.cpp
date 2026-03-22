#include "config/config_parsing.hpp"

#include <gtest/gtest.h>

using namespace reprojection;

TEST(ConfigConfigParsing, TestParseDataConfig) {
    static constexpr std::string_view data_config{R"(
        file = "/data/TUM-Visual-Inertial-Dataset/dataset-calib-imu4.bag"
    )"};
    toml::table toml{toml::parse(data_config)};

    std::string const file{config::ParseDataConfig(toml)};
    EXPECT_EQ(file, "/data/TUM-Visual-Inertial-Dataset/dataset-calib-imu4.bag");

    static constexpr std::string_view bad_config{R"(
        random_key = 123
    )"};
    toml = toml::parse(bad_config);

    EXPECT_THROW(config::ParseDataConfig(toml), std::runtime_error);
}

TEST(ConfigConfigParsing, TestParseSensorConfig) {
    static constexpr std::string_view sensor_config{R"(
        camera_name = "/cam0/image_raw"
        camera_model = "double_sphere"
    )"};
    toml::table toml{toml::parse(sensor_config)};

    auto const [camera_name, camera_model]{config::ParseSensorConfig(toml)};
    EXPECT_EQ(camera_name, "/cam0/image_raw");
    EXPECT_EQ(camera_model, CameraModel::DoubleSphere);

    static constexpr std::string_view bad_config{R"(
        random_key = 123
    )"};
    toml = toml::parse(bad_config);

    EXPECT_THROW(config::ParseSensorConfig(toml), std::runtime_error);
}

TEST(ConfigConfigParsing, TestParseSolverConfig) {
    static constexpr std::string_view solver_config{R"(
        minimizer_type = "LINE_SEARCH"
        min_line_search_step_size = 1e-6
    )"};
    toml::table const toml{toml::parse(solver_config)};

    ceres::Solver::Options const config{config::ParseSolverConfig(toml)};
    EXPECT_EQ(config.minimizer_type, ceres::LINE_SEARCH);
    EXPECT_EQ(config.min_line_search_step_size, 1e-6);
}
