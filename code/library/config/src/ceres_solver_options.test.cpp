#include "ceres_solver_options.hpp"

#include <ceres/solver.h>
#include <ceres/types.h>
#include <gtest/gtest.h>

#include <string_view>
#include <toml++/toml.hpp>

#include "enum_parsing.hpp"

namespace reprojection::config {

// https://stackoverflow.com/questions/257418/do-while-0-what-is-it-good-for
#define CFG_GET_AND_ERASE(name, cfg, options, type)    \
    do {                                               \
        if (auto const value{(cfg)->get(#name)}) {     \
            (options).name = value->as<type>()->get(); \
            (cfg)->erase(#name);                       \
        }                                              \
    } while (0)

// TODO(Jack): We need a better policy here! We should inform the user if the type is wrong, not just silently fail.
#define CFG_GET_ENUM_AND_ERASE(name, cfg, options, enum_type, string_to_enum)                         \
    do {                                                                                              \
        if (auto const value{(cfg)->get_as<std::string>(#name)}) {                                    \
            (options).name = CeresEnumToString<enum_type, string_to_enum>(value->as_string()->get()); \
            (cfg)->erase(#name);                                                                      \
        }                                                                                             \
    } while (0)

// NOTE(Jack): We ignore parameters which would require intimate knowledge of the solver which we do not have. For
// example it is not possible to set the callback pointers from a configuration file obviously...
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

    CFG_GET_ENUM_AND_ERASE(minimizer_type, solver_cfg, options, ceres::MinimizerType, ceres::StringToMinimizerType);
    CFG_GET_ENUM_AND_ERASE(line_search_direction_type, solver_cfg, options, ceres::LineSearchDirectionType,
                           ceres::StringToLineSearchDirectionType);
    CFG_GET_ENUM_AND_ERASE(line_search_type, solver_cfg, options, ceres::LineSearchType, ceres::StringToLineSearchType);
    CFG_GET_ENUM_AND_ERASE(nonlinear_conjugate_gradient_type, solver_cfg, options,
                           ceres::NonlinearConjugateGradientType, ceres::StringToNonlinearConjugateGradientType);
    CFG_GET_AND_ERASE(max_lbfgs_rank, solver_cfg, options, std::int64_t);
    CFG_GET_AND_ERASE(use_approximate_eigenvalue_bfgs_scaling, solver_cfg, options, bool);
    CFG_GET_ENUM_AND_ERASE(line_search_interpolation_type, solver_cfg, options, ceres::LineSearchInterpolationType,
                           ceres::StringToLineSearchInterpolationType);
    CFG_GET_AND_ERASE(min_line_search_step_size, solver_cfg, options, double);

    // Line search parameters
    CFG_GET_AND_ERASE(line_search_sufficient_function_decrease, solver_cfg, options, double);
    CFG_GET_AND_ERASE(max_line_search_step_contraction, solver_cfg, options, double);
    CFG_GET_AND_ERASE(min_line_search_step_contraction, solver_cfg, options, double);
    CFG_GET_AND_ERASE(max_num_line_search_step_size_iterations, solver_cfg, options, std::int64_t);
    CFG_GET_AND_ERASE(max_num_line_search_direction_restarts, solver_cfg, options, std::int64_t);
    CFG_GET_AND_ERASE(line_search_sufficient_curvature_decrease, solver_cfg, options, double);
    CFG_GET_AND_ERASE(max_line_search_step_expansion, solver_cfg, options, double);
    CFG_GET_ENUM_AND_ERASE(trust_region_strategy_type, solver_cfg, options, ceres::TrustRegionStrategyType,
                           ceres::StringToTrustRegionStrategyType);
    CFG_GET_ENUM_AND_ERASE(dogleg_type, solver_cfg, options, ceres::DoglegType, ceres::StringToDoglegType);
    CFG_GET_AND_ERASE(use_nonmonotonic_steps, solver_cfg, options, bool);
    CFG_GET_AND_ERASE(max_consecutive_nonmonotonic_steps, solver_cfg, options, std::int64_t);
    CFG_GET_AND_ERASE(max_num_iterations, solver_cfg, options, std::int64_t);
    CFG_GET_AND_ERASE(max_solver_time_in_seconds, solver_cfg, options, double);
    CFG_GET_AND_ERASE(num_threads, solver_cfg, options, std::int64_t);

    // Trust region minimizer settings.
    CFG_GET_AND_ERASE(initial_trust_region_radius, solver_cfg, options, double);
    CFG_GET_AND_ERASE(max_trust_region_radius, solver_cfg, options, double);
    CFG_GET_AND_ERASE(min_trust_region_radius, solver_cfg, options, double);
    CFG_GET_AND_ERASE(min_relative_decrease, solver_cfg, options, double);
    CFG_GET_AND_ERASE(min_lm_diagonal, solver_cfg, options, double);
    CFG_GET_AND_ERASE(max_lm_diagonal, solver_cfg, options, double);
    CFG_GET_AND_ERASE(max_num_consecutive_invalid_steps, solver_cfg, options, std::int64_t);
    CFG_GET_AND_ERASE(function_tolerance, solver_cfg, options, double);
    CFG_GET_AND_ERASE(gradient_tolerance, solver_cfg, options, double);
    CFG_GET_AND_ERASE(parameter_tolerance, solver_cfg, options, double);

    // Linear least squares solver options
    CFG_GET_ENUM_AND_ERASE(linear_solver_type, solver_cfg, options, ceres::LinearSolverType,
                           ceres::StringToLinearSolverType);
    CFG_GET_ENUM_AND_ERASE(preconditioner_type, solver_cfg, options, ceres::PreconditionerType,
                           ceres::StringToPreconditionerType);
    CFG_GET_ENUM_AND_ERASE(visibility_clustering_type, solver_cfg, options, ceres::VisibilityClusteringType,
                           ceres::StringToVisibilityClusteringType);
    // IGNORED - options.residual_blocks_for_subset_preconditioner
    CFG_GET_ENUM_AND_ERASE(dense_linear_algebra_library_type, solver_cfg, options, ceres::DenseLinearAlgebraLibraryType,
                           ceres::StringToDenseLinearAlgebraLibraryType);
    CFG_GET_ENUM_AND_ERASE(sparse_linear_algebra_library_type, solver_cfg, options,
                           ceres::SparseLinearAlgebraLibraryType, ceres::StringToSparseLinearAlgebraLibraryType);
    CFG_GET_ENUM_AND_ERASE(linear_solver_ordering_type, solver_cfg, options, ceres::LinearSolverOrderingType,
                           ceres::StringToLinearSolverOrderingType);
    // IGNORED - options.linear_solver_ordering
    CFG_GET_AND_ERASE(use_explicit_schur_complement, solver_cfg, options, bool);
    CFG_GET_AND_ERASE(dynamic_sparsity, solver_cfg, options, bool);
    CFG_GET_AND_ERASE(use_mixed_precision_solves, solver_cfg, options, bool);
    CFG_GET_AND_ERASE(max_num_refinement_iterations, solver_cfg, options, std::int64_t);
    CFG_GET_AND_ERASE(min_linear_solver_iterations, solver_cfg, options, std::int64_t);
    CFG_GET_AND_ERASE(max_linear_solver_iterations, solver_cfg, options, std::int64_t);
    CFG_GET_AND_ERASE(max_num_spse_iterations, solver_cfg, options, std::int64_t);
    CFG_GET_AND_ERASE(use_spse_initialization, solver_cfg, options, bool);
    CFG_GET_AND_ERASE(spse_tolerance, solver_cfg, options, double);
    CFG_GET_AND_ERASE(eta, solver_cfg, options, double);
    CFG_GET_AND_ERASE(jacobi_scaling, solver_cfg, options, bool);
    CFG_GET_AND_ERASE(use_inner_iterations, solver_cfg, options, bool);
    // IGNORED - options.inner_iteration_ordering
    CFG_GET_AND_ERASE(inner_iteration_tolerance, solver_cfg, options, double);

    // Logging settings.
    CFG_GET_ENUM_AND_ERASE(logging_type, solver_cfg, options, ceres::LoggingType, ceres::StringtoLoggingType);
    CFG_GET_AND_ERASE(minimizer_progress_to_stdout, solver_cfg, options, bool);
    // IGNORED - options.trust_region_minimizer_iterations_to_dump
    CFG_GET_AND_ERASE(trust_region_problem_dump_directory, solver_cfg, options, std::string);
    // NOTE(Jack): Ceres defines two overloaded versions of StringtoDumpFormatType. With our current implementation of
    // CeresEnumToString we cannot disambiguate them. Therefore, here we need to explicitly wrap the version we want
    // (the one that uses DumpFormatType not LoggingType), and pass that as the template argument.
    auto constexpr StringToDumpFormatTypeHandler = [](std::string const& value, ceres::DumpFormatType* out) {
        return ceres::StringtoDumpFormatType(value, out);
    };
    CFG_GET_ENUM_AND_ERASE(trust_region_problem_dump_format_type, solver_cfg, options, ceres::DumpFormatType,
                           StringToDumpFormatTypeHandler);

    // Finite differences options.
    CFG_GET_AND_ERASE(check_gradients, solver_cfg, options, bool);
    CFG_GET_AND_ERASE(gradient_check_relative_precision, solver_cfg, options, double);
    CFG_GET_AND_ERASE(gradient_check_numeric_derivative_relative_step_size, solver_cfg, options, double);
    CFG_GET_AND_ERASE(update_state_every_iteration, solver_cfg, options, bool);
    // IGNORED - options.callbacks

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

TEST(ConfigCeresSolverOptions, TestLoadSolverOptionsEnums) {
    static constexpr std::string_view config_file{R"(
        [solver]
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
        [solver]
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