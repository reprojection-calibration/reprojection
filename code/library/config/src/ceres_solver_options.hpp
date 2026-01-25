#pragma once

#include <ceres/solver.h>

#include <toml++/toml.hpp>

namespace reprojection::config {

ceres::Solver::Options ParseSolverOptions(toml::table cfg);

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

}  // namespace reprojection::config