#include "ceres_solver_options.hpp"

#include <ceres/solver.h>
#include <ceres/types.h>
#include <gtest/gtest.h>

#include <string_view>
#include <toml++/toml.hpp>

#include "enum_parsing.hpp"

namespace reprojection::config {

// https://stackoverflow.com/questions/257418/do-while-0-what-is-it-good-for
#define CFG_GET_AND_ERASE(cfg, options, type, name)    \
    do {                                               \
        if (auto const value{(cfg)->get(#name)}) {     \
            (options).name = value->as<type>()->get(); \
            (cfg)->erase(#name);                       \
        }                                              \
    } while (0)

ceres::Solver::Options ParseSolverOptions(toml::table cfg) {
    // NOTE(Jack): The options struct is initialized with default values. That means that if in the following code a
    // value is not explicitly set, that its value is preserved as the default.
    ceres::Solver::Options options;

    // TODO(Jack): Should this maybe happen at one level above this function? And if this is true we instead just do not
    // call ParseSolverOptions at all?
    auto* const solver_cfg{cfg["solver"].as_table()};
    if (not solver_cfg) {
        return options;
    }

    // TODO(Jack): Write a macro that automatically generates this code for the simple parameter case.
    // TODO(Jack): We need a better policy here! We should inform the user if the type is wrong, not just silently fail.
    if (auto const value{solver_cfg->get_as<std::string>("minimizer_type")}) {
        options.minimizer_type =
            CeresEnumToString<ceres::MinimizerType, ceres::StringToMinimizerType>(value->as_string()->get());
        solver_cfg->erase("minimizer_type");
    }

    if (auto const value{solver_cfg->get_as<std::string>("line_search_direction_type")}) {
        options.line_search_direction_type =
            CeresEnumToString<ceres::LineSearchDirectionType, ceres::StringToLineSearchDirectionType>(
                value->as_string()->get());
        solver_cfg->erase("line_search_direction_type");
    }

    if (auto const value{solver_cfg->get_as<std::string>("line_search_type")}) {
        options.line_search_type =
            CeresEnumToString<ceres::LineSearchType, ceres::StringToLineSearchType>(value->as_string()->get());
        solver_cfg->erase("line_search_type");
    }

    if (auto const value{solver_cfg->get_as<std::string>("nonlinear_conjugate_gradient_type")}) {
        options.nonlinear_conjugate_gradient_type =
            CeresEnumToString<ceres::NonlinearConjugateGradientType, ceres::StringToNonlinearConjugateGradientType>(
                value->as_string()->get());
        solver_cfg->erase("nonlinear_conjugate_gradient_type");
    }

    CFG_GET_AND_ERASE(solver_cfg, options, std::int64_t, max_lbfgs_rank);

    if (solver_cfg->size() != 0) {
        // TODO(Jack): Print the keys and values in the error message
        throw std::runtime_error("Unread keys found in table xxxx!!!!");
    }

    return options;
}
}  // namespace reprojection::config

using namespace reprojection;
using namespace std::string_view_literals;

// Given a table without the [solver] heading we will just get back the default configuration from ceres
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
// leftover key at the end of parsing which is an error we throw on.
TEST(ConfigCeresSolverOptions, TestLoadSolverOptionsMinimizerTypeWrongType) {
    static constexpr std::string_view config_file{R"(
        [solver]
        minimizer_type = 101.1
    )"sv};
    toml::table const config{toml::parse(config_file)};

    EXPECT_THROW(config::ParseSolverOptions(config), std::runtime_error);
}

TEST(ConfigCeresSolverOptions, TestLoadSolverOptionsMaxLbfgsRankInt) {
    static constexpr std::string_view config_file{R"(
        [solver]
        max_lbfgs_rank = 21
    )"sv};
    toml::table const config{toml::parse(config_file)};

    auto const solver_options{config::ParseSolverOptions(config)};
    EXPECT_EQ(solver_options.max_lbfgs_rank, 21);
}