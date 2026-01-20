#include "optimization/nonlinear_refinement.hpp"

#include "eigen_utilities/grid.hpp"
#include "projection_cost_function.hpp"

namespace reprojection::optimization {

// TODO(Jack): Return report summary of optimization.
// TODO(Jack): Use the valid cost function mask to visualize only the valid pixels in the dashboard.
//  Calculate initial reprojection error and get the valid cost function mask.
// ERROR(Jack): What is a frame has too few valid pixels to actually constrain the pose? Should we entirely skip
// that frame? Or what if in general we have a minimum required of points per frame threshold?
// TODO(Jack): What should we do with the mask from the optimized reprojection error calculation? Does it have meaning
//  for us?
void CameraNonlinearRefinement(OptimizationDataView data_view) {
    std::map<uint64_t, std::vector<std::unique_ptr<ceres::CostFunction>>> cost_functions;
    for (OptimizationFrameView const& frame_i : data_view) {
        MatrixX2d const& pixels_i{frame_i.extracted_target().bundle.pixels};
        MatrixX3d const& points_i{frame_i.extracted_target().bundle.points};

        for (Eigen::Index j{0}; j < pixels_i.rows(); ++j) {
            cost_functions[frame_i.timestamp_ns()].emplace_back(
                Create(data_view.camera_model(), data_view.image_bounds(), pixels_i.row(j), points_i.row(j)));
        }
    }

    ceres::Problem::Options problem_options;
    problem_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    ceres::Problem problem{problem_options};
    for (OptimizationFrameView frame_i : data_view) {
        auto const& cost_functions_i{cost_functions.at(frame_i.timestamp_ns())};

        ArrayXb mask;
        std::tie(frame_i.initial_reprojection_error(), mask) =
            EvaluateReprojectionResiduals(cost_functions_i, data_view.initial_intrinsics(), frame_i.initial_pose());
        ArrayXi const valid_ids{eigen_utilities::MaskToRowId(mask)};
        for (int i{0}; i < valid_ids.rows(); ++i) {
            problem.AddResidualBlock(cost_functions_i[valid_ids(i)].get(), nullptr,
                                     data_view.optimized_intrinsics().data(), frame_i.optimized_pose().value().data());
        }
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // Calculate optimized reprojection error.
    for (OptimizationFrameView frame_i : data_view) {
        ArrayXb what_to_do;
        std::tie(frame_i.optimized_reprojection_error(), what_to_do) =
            EvaluateReprojectionResiduals(cost_functions.at(frame_i.timestamp_ns()), data_view.optimized_intrinsics(),
                                          frame_i.optimized_pose().value());
    }
}

std::tuple<ArrayX2d, ArrayXb> EvaluateReprojectionResiduals(
    std::vector<std::unique_ptr<ceres::CostFunction>> const& cost_functions, ArrayXd const& intrinsics,
    Array6d const& pose) {
    std::vector<double const*> parameter_blocks;
    parameter_blocks.push_back(intrinsics.data());
    parameter_blocks.push_back(pose.data());

    // NOTE(Jack): Eigen is column major by default. Which means that if you just make a default array here and pass the
    // row pointer blindly into the EvaluateResidualBlock function it will not fill out the row but actually two column
    // elements! That is the reason why we have to specifically specify RowMajor here!
    Eigen::Array<double, Eigen::Dynamic, 2, Eigen::RowMajor> residuals{std::size(cost_functions), 2};
    ArrayXb valid_mask{ArrayXb::Zero(std::size(cost_functions), 1)};
    for (size_t i{0}; i < std::size(cost_functions); ++i) {
        valid_mask(i) = cost_functions[i]->Evaluate(parameter_blocks.data(), residuals.row(i).data(), nullptr);
    }

    return {residuals, valid_mask};
}  // LCOV_EXCL_LINE

// TODO(Jack): Naming!
CubicBSplineC3Refinement::CubicBSplineC3Refinement(spline::CubicBSplineC3 const& spline) : spline_{spline} {}

ceres::Solver::Summary CubicBSplineC3Refinement::Solve() {
    ceres::Solver::Options options;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem_, &summary);

    return summary;
}

spline::CubicBSplineC3 CubicBSplineC3Refinement::GetSpline() const { return spline_; }

}  // namespace  reprojection::optimization
