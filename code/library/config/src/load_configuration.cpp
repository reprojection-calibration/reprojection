#include "config/load_configuration.hpp"

#include <toml++/toml.hpp>

#include "ceres_solver_options.hpp"

namespace reprojection::config {

// NOTE(Jack): As we add more configs we will probably return a tuple here or a struct with all the possible configs.
ceres::Solver::Options LoadConfiguration(std::string const& file) {
    toml::table const cfg{toml::parse_file(file)};

    if (auto solver_node{cfg["solver"]}) {
        return ParseSolverOptions(*solver_node.as_table());
    } else {
        return ceres::Solver::Options{};
    }
}

}  // namespace reprojection::config