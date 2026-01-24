#include "ceres_solver_options.hpp"

#include <ceres/solver.h>
#include <ceres/types.h>
#include <gtest/gtest.h>

#include <string_view>
#include <toml++/toml.hpp>

namespace reprojection::config {

ceres::Solver::Options ParseSolverOptions(toml::table const& config) {
    // NOTE(Jack): The options struct is initialized with default values. That means that if in the following code a
    // value is not explicitly set, that its value is preserved as the default.
    ceres::Solver::Options options;

    auto const* const solver_config{config["solver"].as_table()};
    if (not solver_config) {
        return options;
    }

    return options;
}
}  // namespace reprojection::config

using namespace reprojection;
using namespace std::string_view_literals;

TEST(ConfigCeresSolverOptions, TestLoadSolverOptionsDefault) {
    static constexpr std::string_view some_toml{R"(
        [ceres]
        [ceres.not_the_solver_config]
    )"sv};
    toml::table const config{toml::parse(some_toml)};

    auto const solver_options{config::ParseSolverOptions(config)};
    std::string error_msg;
    EXPECT_TRUE(solver_options.IsValid(&error_msg));
    EXPECT_EQ(std::size(error_msg), 0);
}