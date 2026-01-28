#include "config/load_config_file.hpp"

#include <gtest/gtest.h>

#include "testing_utilities/temporary_file.hpp"

using namespace reprojection;
using TemporaryFile = testing_utilities::TemporaryFile;

TEST(ConfigLoadConfigFile, TestLoadConfigFile) {
    static constexpr std::string_view happy_path_config{R"(
        [solver]
        minimizer_type = "LINE_SEARCH"
        min_line_search_step_size = 1e-6
    )"};
    TemporaryFile const config_file{".toml", happy_path_config};

    ceres::Solver::Options const config{config::LoadConfigFile(config_file.Path())};
    EXPECT_EQ(config.minimizer_type, ceres::LINE_SEARCH);
    EXPECT_EQ(config.min_line_search_step_size, 1e-6);
}

// TODO(Jack): This test is too specifically targeted at the solver config - that should not be the case for the general
//  load configuration testing.
//
// There is no [solver] config to load, so we will instead get a default initialized ceres::Solver::Options back :)
TEST(ConfigLoadConfigFile, TestLoadConfigFileDefaultSolverOptions) {
    static constexpr std::string_view no_solver_parameters_config{R"(
        [some_other_config]
        blah = 1
    )"};
    TemporaryFile const config_file{".toml", no_solver_parameters_config};

    ceres::Solver::Options const config{config::LoadConfigFile(config_file.Path())};
    EXPECT_EQ(config.minimizer_type, ceres::TRUST_REGION);
    EXPECT_EQ(config.min_line_search_step_size, 1e-9);
}