#include "optimization/camera_nonlinear_refinement.hpp"

#include "eigen_utilities/grid.hpp"

#include "projection_cost_function.hpp"

namespace reprojection::optimization {

// TODO(Jack): Confirm that OptimizationState initialization is a deep copy!
// ERROR(Jack): What is a frame has too few valid pixels to actually constrain the pose? Should we entirely skip
// that frame? Or what if in general we have a minimum required of points per frame threshold?
std::tuple<OptimizationState, CeresState> CameraNonlinearRefinement(CameraInfo const& sensor,
                                                                    CameraMeasurements const& data,
                                                                    OptimizationState const& initial_state) {
    // Create the ceres problem and configure it - one day the ceres state will be set from a config file!
    CeresState ceres_state{ceres::TAKE_OWNERSHIP, ceres::DENSE_SCHUR};
    ceres::Problem problem{ceres_state.problem_options};

    // ERROR(Jack): Iterate over the initial state here so that if there are data for which there is no pose!!!
    // ERROR(Jack): Iterate over the initial state here so that if there are data for which there is no pose!!!
    // ERROR(Jack): Iterate over the initial state here so that if there are data for which there is no pose!!!
    // ERROR(Jack): Iterate over the initial state here so that if there are data for which there is no pose!!!
    // ERROR(Jack): Iterate over the initial state here so that if there are data for which there is no pose!!!
    // ERROR(Jack): Iterate over the initial state here so that if there are data for which there is no pose!!!
    OptimizationState optimized_state{initial_state};
    for (auto const& [timestamp_ns, target] : data) {
        MatrixX2d const& pixels_i{target.bundle.pixels};
        MatrixX3d const& points_i{target.bundle.points};

        for (Eigen::Index j{0}; j < pixels_i.rows(); ++j) {
            ceres::CostFunction* const cost_function{
                Create(sensor.camera_model, sensor.bounds, pixels_i.row(j), points_i.row(j))};

            problem.AddResidualBlock(cost_function, nullptr, optimized_state.camera_state.intrinsics.data(),
                                     optimized_state.frames.at(timestamp_ns).pose.data());
        }
    }

    // Optimize!!!
    ceres::Solve(ceres_state.solver_options, &problem, &ceres_state.solver_summary);

    return {optimized_state, ceres_state};
}

// TODO CALCULATE THE REPROJECTION ERROR OUTSIDE OF THE MAIN OPTIMIZATiON FUNCTION!!!!!
// TODO CALCULATE THE REPROJECTION ERROR OUTSIDE OF THE MAIN OPTIMIZATiON FUNCTION!!!!!
// TODO CALCULATE THE REPROJECTION ERROR OUTSIDE OF THE MAIN OPTIMIZATiON FUNCTION!!!!!
// TODO CALCULATE THE REPROJECTION ERROR OUTSIDE OF THE MAIN OPTIMIZATiON FUNCTION!!!!!
// TODO CALCULATE THE REPROJECTION ERROR OUTSIDE OF THE MAIN OPTIMIZATiON FUNCTION!!!!!
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

}  // namespace  reprojection::optimization
