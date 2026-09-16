#pragma once

#include <ceres/problem.h>
#include <ceres/solver.h>

namespace reprojection {

struct CeresState {
    CeresState() = default;

    CeresState(ceres::Ownership const ownership, ceres::LinearSolverType const linear_solver, int const num_threads) {
        problem_options.cost_function_ownership = ownership;
        solver_options.linear_solver_type = linear_solver;
        solver_options.num_threads = num_threads;
    }

    ceres::Problem::Options problem_options;
    ceres::Solver::Options solver_options;
    ceres::Solver::Summary solver_summary;
};

}  // namespace reprojection