#pragma once

#include <ceres/solver.h>

#include <toml++/toml.hpp>

namespace reprojection::config {

ceres::Solver::Options ParseSolverOptions(toml::table cfg);

// NOTE(Jack): We use macros here because we need to interface directly with the ceres options struct. Using a macro
// allows us to specify the name of the parameter only once in a single place, and then generates the entire reading
// logic for each param. If there is a better way to do this I am open to suggestions.
// NOTE(Jack): Explanation for the do-while logic
// https://stackoverflow.com/questions/257418/do-while-0-what-is-it-good-for
#define CFG_GET_AND_ERASE(name, cfg, options, type)    \
    do {                                               \
        if (auto const value{(cfg).get(#name)}) {     \
            (options).name = value->as<type>()->get(); \
            (cfg).erase(#name);                       \
        }                                              \
    } while (0)

#define CFG_GET_ENUM_AND_ERASE(name, cfg, options, enum_type, string_to_enum)                         \
    do {                                                                                              \
        if (auto const value{(cfg).get_as<std::string>(#name)}) {                                    \
            (options).name = CeresEnumToString<enum_type, string_to_enum>(value->as_string()->get()); \
            (cfg).erase(#name);                                                                      \
        }                                                                                             \
    } while (0)

}  // namespace reprojection::config