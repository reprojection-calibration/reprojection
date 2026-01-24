#include "ceres_solver_options.hpp"

#include <ceres/solver.h>
#include <ceres/types.h>
#include <gtest/gtest.h>

#include <string_view>
#include <toml++/toml.hpp>

#include "enum_parsing.hpp"

namespace reprojection::config {

ceres::Solver::Options ParseSolverOptions(toml::table config) {
    // NOTE(Jack): The options struct is initialized with default values. That means that if in the following code a
    // value is not explicitly set, that its value is preserved as the default.
    ceres::Solver::Options options;

    auto* const solver_cfg{config["solver"].as_table()};
    if (not solver_cfg) {
        return options;
    }

    // TODO(Jack): We need a better policy here! We should inform the user if the type is wrong, not just silently fail.
    if (auto const value{solver_cfg->get_as<std::string>("minimizer_type")}) {
        solver_cfg->erase("minimizer_type");
        options.minimizer_type =
            CeresEnumToString<ceres::MinimizerType, ceres::StringToMinimizerType>(value->as_string()->get());
    }

    if (solver_cfg->size() != 0) {
        // TODO(Jack): Print the keys and values in the error message
        throw std::runtime_error("Unread keys found in table xxxx!!!!");
    }

    return options;
}
}  // namespace reprojection::config

using namespace reprojection;
using namespace std::string_view_literals;

TEST(ConfigCeresSolverOptions, TestLoadSolverOptionsDefault) {
    static constexpr std::string_view config_file{R"(
        [not_the_solver_config]
    )"sv};
    toml::table const config{toml::parse(config_file)};

    auto const solver_options{config::ParseSolverOptions(config)};
    std::string error_msg;
    EXPECT_TRUE(solver_options.IsValid(&error_msg));
    EXPECT_EQ(std::size(error_msg), 0);
}

TEST(ConfigCeresSolverOptions, TestLoadSolverOptionsMinimizerType) {
    static constexpr std::string_view config_file{R"(
        [solver]
        minimizer_type = "LINE_SEARCH"
    )"sv};
    toml::table const config{toml::parse(config_file)};

    auto const solver_options{config::ParseSolverOptions(config)};
    EXPECT_EQ(solver_options.minimizer_type, ceres::LINE_SEARCH);
}

// Given the wrong type this key/value will not be parsed and removed from the table, which means we will have a
// leftover key at the end of parsing which is an error and we throw on.
TEST(ConfigCeresSolverOptions, TestLoadSolverOptionsMinimizerTypeWrongType) {
    static constexpr std::string_view config_file{R"(
        [solver]
        minimizer_type = 101.1
    )"sv};
    toml::table const config{toml::parse(config_file)};

    EXPECT_THROW(config::ParseSolverOptions(config), std::runtime_error);
}