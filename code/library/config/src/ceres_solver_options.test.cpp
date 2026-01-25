#include "ceres_solver_options.hpp"

#include <ceres/types.h>
#include <gtest/gtest.h>

#include <string_view>

using namespace reprojection;
using namespace std::string_view_literals;

TEST(ConfigCeresSolverOptions, TestLoadSolverOptionsFaultyInput) {
    // Config with a section header we dont expect
    static constexpr std::string_view config_file{R"(
        [not_the_solver_config]
    )"sv};
    toml::table const config{toml::parse(config_file)};

    EXPECT_THROW(config::ParseSolverOptions(config), std::runtime_error);

    // Config with a key value pair we dont expect
    static constexpr std::string_view config_file_1{R"(
        some_random_parameter = 666
    )"sv};
    toml::table const config_1{toml::parse(config_file_1)};

    EXPECT_THROW(config::ParseSolverOptions(config_1), std::runtime_error);
}

// Given the wrong type this key/value will not be parsed and removed from the table, which means we will have a
// leftover key at the end of parsing which is an error we throw on.
TEST(ConfigCeresSolverOptions, TestLoadSolverOptionsMinimizerTypeWrongType) {
    static constexpr std::string_view config_file{R"(
        minimizer_type = 101.1
    )"sv};
    toml::table const config{toml::parse(config_file)};

    EXPECT_THROW(config::ParseSolverOptions(config), std::runtime_error);
}

TEST(ConfigCeresSolverOptions, TestLoadSolverOptionsEnums) {
    static constexpr std::string_view config_file{R"(
        minimizer_type = "LINE_SEARCH"
        line_search_interpolation_type = "QUADRATIC"
        trust_region_problem_dump_format_type = "CONSOLE"
    )"sv};
    toml::table const config{toml::parse(config_file)};

    auto const solver_options{config::ParseSolverOptions(config)};
    EXPECT_EQ(solver_options.minimizer_type, ceres::LINE_SEARCH);
    EXPECT_EQ(solver_options.line_search_interpolation_type, ceres::QUADRATIC);
    EXPECT_EQ(solver_options.trust_region_problem_dump_format_type, ceres::CONSOLE);
}

// Test all the non-enum types we have - int, bool, double, std::string
TEST(ConfigCeresSolverOptions, TestLoadSolverOptionsMaxLbfgsRankInt) {
    static constexpr std::string_view config_file{R"(
        max_lbfgs_rank = 21
        use_approximate_eigenvalue_bfgs_scaling = true
        min_line_search_step_size = 1e-6
        trust_region_problem_dump_directory = "/my/log/directory"
    )"sv};
    toml::table const config{toml::parse(config_file)};

    auto const solver_options{config::ParseSolverOptions(config)};
    EXPECT_EQ(solver_options.max_lbfgs_rank, 21);
    EXPECT_EQ(solver_options.use_approximate_eigenvalue_bfgs_scaling, true);
    EXPECT_EQ(solver_options.min_line_search_step_size, 1e-6);
    EXPECT_EQ(solver_options.trust_region_problem_dump_directory, "/my/log/directory");
}