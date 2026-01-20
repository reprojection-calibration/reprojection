#include "optimization/nonlinear_refinement.hpp"

#include "eigen_utilities/grid.hpp"
#include "projection_cost_function.hpp"

namespace reprojection::optimization {

// ERROR(Jack): What is a frame has too few valid pixels to actually constrain the pose? Should we entirely skip
// that frame? Or what if in general we have a minimum required of points per frame threshold?
CeresState CameraNonlinearRefinement(OptimizationDataView data_view) {
    // Assemble all cost functions.
    std::map<uint64_t, std::vector<std::unique_ptr<ceres::CostFunction>>> cost_functions;
    for (OptimizationFrameView const& frame_i : data_view) {
        MatrixX2d const& pixels_i{frame_i.extracted_target().bundle.pixels};
        MatrixX3d const& points_i{frame_i.extracted_target().bundle.points};

        for (Eigen::Index j{0}; j < pixels_i.rows(); ++j) {
            cost_functions[frame_i.timestamp_ns()].emplace_back(
                Create(data_view.camera_model(), data_view.image_bounds(), pixels_i.row(j), points_i.row(j)));
        }
    }

    // Create the ceres problem and configure it - one day this will happen from a config file!
    CeresState ceres_state{ceres::DO_NOT_TAKE_OWNERSHIP, ceres::DENSE_SCHUR};
    ceres::Problem problem{ceres_state.problem_options};

    // Calculate the reprojection errors before optimization and then add all cost functions to the problem.
    for (OptimizationFrameView frame_i : data_view) {
        auto const& cost_functions_i{cost_functions.at(frame_i.timestamp_ns())};

        frame_i.initial_reprojection_error() =
            EvaluateReprojectionResiduals(cost_functions_i, data_view.initial_intrinsics(), frame_i.initial_pose());

        for (auto const& cost_function : cost_functions_i) {
            problem.AddResidualBlock(cost_function.get(), nullptr, data_view.optimized_intrinsics().data(),
                                     frame_i.optimized_pose().value().data());
        }
    }

    // Optimize!!!
    ceres::Solve(ceres_state.solver_options, &problem, &ceres_state.solver_summary);

    // Calculate the reprojection errors after optimization.
    for (OptimizationFrameView frame_i : data_view) {
        // cppcheck-suppress unreadVariable
        frame_i.optimized_reprojection_error() =
            EvaluateReprojectionResiduals(cost_functions.at(frame_i.timestamp_ns()), data_view.optimized_intrinsics(),
                                          frame_i.optimized_pose().value());
    }

    return ceres_state;
}

ArrayX2d EvaluateReprojectionResiduals(std::vector<std::unique_ptr<ceres::CostFunction>> const& cost_functions,
                                       ArrayXd const& intrinsics, Array6d const& pose) {
    std::vector<double const*> parameter_blocks;
    parameter_blocks.push_back(intrinsics.data());
    parameter_blocks.push_back(pose.data());

    // NOTE(Jack): Eigen is column major by default. Which means that if you just make a default array here and pass the
    // row pointer blindly into the EvaluateResidualBlock function it will not fill out the row but actually two column
    // elements! That is the reason why we have to specifically specify RowMajor here!
    Eigen::Array<double, Eigen::Dynamic, 2, Eigen::RowMajor> residuals{std::size(cost_functions), 2};
    for (size_t i{0}; i < std::size(cost_functions); ++i) {
        cost_functions[i]->Evaluate(parameter_blocks.data(), residuals.row(i).data(), nullptr);
    }

    return residuals;
}  // LCOV_EXCL_LINE

// TODO(Jack): Naming!
CubicBSplineC3Refinement::CubicBSplineC3Refinement(spline::CubicBSplineC3 const& spline) : spline_{spline} {}

ceres::Solver::Summary CubicBSplineC3Refinement::Solve() {
    // TODO(Jack): Use the CeresState used by the camera optimization.
    ceres::Solver::Options options;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem_, &summary);

    return summary;
}

spline::CubicBSplineC3 CubicBSplineC3Refinement::GetSpline() const { return spline_; }

}  // namespace  reprojection::optimization
