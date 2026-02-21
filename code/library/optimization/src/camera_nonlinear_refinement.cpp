#include "optimization/camera_nonlinear_refinement.hpp"

#include <ranges>

#include "projection_cost_function.hpp"

namespace reprojection::optimization {

// TODO(Jack): Confirm that OptimizationState initialization is a deep copy!
// ERROR(Jack): What is a frame has too few valid pixels to actually constrain the pose? Should we entirely skip
// that frame? Or what if in general we have a minimum required of points per frame threshold?
std::tuple<OptimizationState, CeresState> CameraNonlinearRefinement(CameraInfo const& sensor,
                                                                    CameraMeasurements const& targets,
                                                                    OptimizationState const& initial_state) {
    CeresState ceres_state{ceres::TAKE_OWNERSHIP, ceres::DENSE_SCHUR};
    ceres::Problem problem{ceres_state.problem_options};

    OptimizationState optimized_state{initial_state};
    for (auto const timestamp_ns : optimized_state.frames | std::views::keys) {
        auto const& [pixels, points]{targets.at(timestamp_ns).bundle};

        for (Eigen::Index j{0}; j < pixels.rows(); ++j) {
            ceres::CostFunction* const cost_function{
                Create(sensor.camera_model, sensor.bounds, pixels.row(j), points.row(j))};

            problem.AddResidualBlock(cost_function, nullptr, optimized_state.camera_state.intrinsics.data(),
                                     optimized_state.frames.at(timestamp_ns).pose.data());
        }
    }

    ceres::Solve(ceres_state.solver_options, &problem, &ceres_state.solver_summary);

    return {optimized_state, ceres_state};
}

ReprojectionErrors ReprojectionResiduals(CameraInfo const& sensor, CameraMeasurements const& targets,
                                         OptimizationState const& state) {
    ReprojectionErrors residuals;
    for (auto const& [timestamp_ns, frame_i] : state.frames) {
        auto const& [pixels, points]{targets.at(timestamp_ns).bundle};

        std::vector<double const*> parameter_blocks;
        parameter_blocks.push_back(state.camera_state.intrinsics.data());
        parameter_blocks.push_back(frame_i.pose.data());

        // NOTE(Jack): Eigen is column major by default. Which means that if you just make a default array here and pass
        // the row pointer blindly into the EvaluateResidualBlock function it will not fill out the row but actually two
        // column elements! That is the reason why we have to specifically specify RowMajor here!
        Eigen::Array<double, Eigen::Dynamic, 2, Eigen::RowMajor> residuals_i{pixels.rows(), 2};
        for (Eigen::Index i{0}; i < pixels.rows(); ++i) {
            ceres::CostFunction const* const cost_function{
                Create(sensor.camera_model, sensor.bounds, pixels.row(i), points.row(i))};

            cost_function->Evaluate(parameter_blocks.data(), residuals_i.row(i).data(), nullptr);
        }

        residuals.insert({timestamp_ns, residuals_i});
    }

    return residuals;
}  // LCOV_EXCL_LINE

}  // namespace  reprojection::optimization
