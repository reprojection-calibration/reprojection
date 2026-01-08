#include "optimization/nonlinear_refinement.hpp"

#include "geometry/lie.hpp"
#include "projection_cost_function.hpp"

namespace reprojection::optimization {

// TODO(Jack): Should we have some assertions which force that the frames satisfy some basic properties like there is a
// matching number of everything? Yes but maybe this is already to deep into the code to do that here.
// TODO(Jack): Refactor to accept se3 pose directly? No! Use quaterions!
void CameraNonlinearRefinement(OptimizationDataView data_view) {
    // Setting the "optimized" value here and below (i.e. optimized_pose) is how we initialize the values with the
    // initial value and which are then used and optimized in place by the solver.
    data_view.optimized_intrinsics() = data_view.initial_intrinsics();

    ceres::Problem problem;
    for (OptimizationFrameView frame_i : data_view) {
        MatrixX2d const& pixels_i{frame_i.extracted_target().bundle.pixels};
        MatrixX3d const& points_i{frame_i.extracted_target().bundle.points};
        frame_i.optimized_pose() = frame_i.initial_pose();

        for (Eigen::Index j{0}; j < pixels_i.rows(); ++j) {
            ceres::CostFunction* const cost_function{
                Create(data_view.camera_model(), pixels_i.row(j), points_i.row(j))};

            problem.AddResidualBlock(cost_function, nullptr, data_view.optimized_intrinsics().data(),
                                     frame_i.optimized_pose().data());
        }
    }

    // TODO(Jack): Law of useful return states that we should probably be returning the summary!
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
}

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
