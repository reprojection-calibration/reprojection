#pragma once

#include <ceres/problem.h>
#include <ceres/solver.h>

namespace reprojection {

struct CeresState {
    CeresState() = default;

    CeresState(ceres::Ownership const ownership, ceres::LinearSolverType const linear_solver) {
        problem_options.cost_function_ownership = ownership;
        solver_options.linear_solver_type = linear_solver;
    }

    ceres::Problem::Options problem_options;
    ceres::Solver::Options solver_options;
    ceres::Solver::Summary solver_summary;
};

}  // namespace reprojection