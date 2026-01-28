#include "config/load_config_file.hpp"

#include <toml++/toml.hpp>

#include "parse_ceres_solver_options.hpp"

namespace reprojection::config {

// NOTE(Jack): As we add more configs we will probably return a tuple here or a struct with all the possible configs.
ceres::Solver::Options LoadConfigFile(std::string const& file) {
    toml::table const cfg{toml::parse_file(file)};

    // Sensible default is available here! Not the case for the target config!
    if (auto const solver_node{cfg["solver"]}) {
        return ParseCeresSolverOptions(*solver_node.as_table());
    } else {
        return ceres::Solver::Options{};
    }
}

}  // namespace reprojection::config