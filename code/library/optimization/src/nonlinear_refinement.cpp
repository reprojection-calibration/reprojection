#include "optimization/nonlinear_refinement.hpp"

#include "geometry/lie.hpp"
#include "projection_cost_function.hpp"

namespace reprojection::optimization {

// TODO(Jack): Should we have some assertions which force that the frames satisfy some basic properties like there is a
// matching number of everything? Yes but maybe this is already to deep into the code to do that here.
void CameraNonlinearRefinement(OptimizationDataView data_view) {
    data_view.optimized_intrinsics() = data_view.initial_intrinsics();

    ceres::Problem problem;
    std::map<uint64_t, std::vector<ceres::ResidualBlockId>> residual_id_map;  // TODO(Jack): Naming?
    for (OptimizationFrameView frame_i : data_view) {
        MatrixX2d const& pixels_i{frame_i.extracted_target().bundle.pixels};
        MatrixX3d const& points_i{frame_i.extracted_target().bundle.points};
        frame_i.optimized_pose() = frame_i.initial_pose();

        std::vector<ceres::ResidualBlockId> residual_ids_i;  // TODO(Jack): Naming?
        for (Eigen::Index j{0}; j < pixels_i.rows(); ++j) {
            ceres::CostFunction* const cost_function{
                Create(data_view.camera_model(), data_view.image_bounds(), pixels_i.row(j), points_i.row(j))};

            ceres::ResidualBlockId const id{problem.AddResidualBlock(
                cost_function, nullptr, data_view.optimized_intrinsics().data(), frame_i.optimized_pose().data())};
            residual_ids_i.push_back(id);
        }
        residual_id_map[frame_i.timestamp_ns()] = residual_ids_i;
    }

    for (OptimizationFrameView frame_i : data_view) {
        frame_i.initial_reprojection_error() =
            EvaluateReprojectionResiduals(problem, residual_id_map[frame_i.timestamp_ns()]);
    }

    // TODO(Jack): Law of useful return states that we should probably be returning the summary!
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    for (OptimizationFrameView frame_i : data_view) {
        frame_i.optimized_reprojection_error() =
            EvaluateReprojectionResiduals(problem, residual_id_map[frame_i.timestamp_ns()]);
    }
}

ArrayX2d EvaluateReprojectionResiduals(ceres::Problem const& problem,
                                       std::vector<ceres::ResidualBlockId> const& residual_ids) {
    // WARN(Jack): Eigen is column major by default. Which means that if you just make a default array here and pass the
    // row pointer blindly into the EvaluateResidualBlock function it will not fill out the row but actually two column
    // elements! That is the reason why we have to specifically specify RowMajor here!
    Eigen::Array<double, Eigen::Dynamic, 2, Eigen::RowMajor> residuals{std::size(residual_ids), 2};
    for (size_t i{0}; i < std::size(residual_ids); ++i) {
        bool const success{
            problem.EvaluateResidualBlock(residual_ids[i], false, nullptr, residuals.row(i).data(), nullptr)};
        std::cout << success << std::endl;
    }

    return residuals;
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
