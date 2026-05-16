#include "config/config_parsing.hpp"

#include <gtest/gtest.h>

#include "testing_utilities/generated/minimum_config.hpp"

using namespace reprojection;

TEST(ConfigConfigParsing, TestParseSensorConfig) {
    toml::table toml{toml::parse(testing_utilities::minimum_config)};

    auto const [camera_name, camera_model]{config::ParseSensorConfig(*toml["sensor"].as_table())};
    EXPECT_EQ(camera_name, "/cam0/image_raw");
    EXPECT_EQ(camera_model, CameraModel::DoubleSphere);

    static constexpr std::string_view bad_config{R"(
        random_key = 123
    )"};
    toml = toml::parse(bad_config);
    EXPECT_THROW(config::ParseSensorConfig(toml), std::runtime_error);
}

TEST(ConfigConfigParsing, TestParseTargetConfig) {
    static constexpr std::string_view target_config{R"(
        pattern_size = [3,4]
        type = "aprilgrid3"
        unit_dimension = 0.1

        [circle_grid]
        asymmetric = true
    )"};
    toml::table toml{toml::parse(target_config)};

    TargetInfo const target_info{config::ParseTargetConfig(toml)};
    EXPECT_EQ(target_info.target_type, TargetType::Aprilgrid3);
    EXPECT_EQ(target_info.height, 3);
    EXPECT_EQ(target_info.width, 4);
    EXPECT_EQ(target_info.unit_dimension, 0.1);
    EXPECT_EQ(target_info.asymmetric, true);

    static constexpr std::string_view bad_config{R"(
        random_key = 123
    )"};
    toml = toml::parse(bad_config);
    EXPECT_THROW(config::ParseTargetConfig(toml), std::runtime_error);
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
