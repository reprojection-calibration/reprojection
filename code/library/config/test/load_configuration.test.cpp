#include "config/load_configuration.hpp"

#include <gtest/gtest.h>

using namespace reprojection;

TEST(ConfigLoadConfiguration, TestLoadConfiguration) {
    // WARN(Jack): If you make a change to the config you need to regenerate the cmake!
    //
    // This is the classic "where is my test actually running from", and what is its absolute path problem, which makes
    // it tricky to load files from the host system in a generic way. We solve this problem for the test_config.toml by
    // copying it to the ${CMAKE_CURRENT_BINARY_DIR} during build configuration time. See the note in the cmakelists for
    // more context.
    std::string const config_file{"./test_config.toml"};

    ceres::Solver::Options const config{config::LoadConfiguration(config_file)};

    EXPECT_EQ(config.minimizer_type, ceres::LINE_SEARCH);
    EXPECT_EQ(config.min_line_search_step_size, 1e-6);
}