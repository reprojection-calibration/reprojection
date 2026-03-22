#include "config/config_parsing.hpp"

#include <gtest/gtest.h>


using namespace reprojection;

TEST(ConfigConfigParsing, TestParseSolverConfig) {
    static constexpr std::string_view solve_config{R"(
        minimizer_type = "LINE_SEARCH"
        min_line_search_step_size = 1e-6
    )"};
    toml::table const toml{toml::parse(solve_config)};

    ceres::Solver::Options const config{config::ParseSolverConfig(toml)};
    EXPECT_EQ(config.minimizer_type, ceres::LINE_SEARCH);
    EXPECT_EQ(config.min_line_search_step_size, 1e-6);
}

