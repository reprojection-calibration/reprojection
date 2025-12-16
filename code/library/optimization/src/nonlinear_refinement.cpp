#include "optimization/nonlinear_refinement.hpp"

#include "geometry/lie.hpp"
#include "projection_cost_function.hpp"

namespace reprojection::optimization {

// TODO(Jack): Should we have some assertions which force that the frames satisfy some basic properties like there is a
// matching number of everything?
// TODO(Jack): Refactor to accept se3 pose directly?
std::tuple<std::vector<Isometry3d>, ArrayXd, double> CameraNonlinearRefinement(std::vector<Frame> const& frames,
                                                                               CameraModel const& camera_type,
                                                                               ArrayXd const& intrinsics) {
    assert(static_cast<int>(camera_type) == intrinsics.rows());  // This is not a real error handling strategy!

    std::vector<Array6d> se3;
    se3.reserve(std::size(frames));
    ArrayXd intrinsics_to_optimize{intrinsics};  // Same intrinsics for ALL poses - mono camera constraint

    ceres::Problem problem;
    for (size_t i{0}; i < std::size(frames); ++i) {
        MatrixX2d const& pixels_i{frames[i].bundle.pixels};
        MatrixX3d const& points_i{frames[i].bundle.points};
        se3.push_back(geometry::Log(frames[i].pose));

        for (Eigen::Index j{0}; j < pixels_i.rows(); ++j) {
            ceres::CostFunction* const cost_function{Create(camera_type, pixels_i.row(j), points_i.row(j))};

            problem.AddResidualBlock(cost_function, nullptr, intrinsics_to_optimize.data(), se3[i].data());
        }
    }

    // TODO(Jack): Law of useful return states that we should probably be returning the summary!
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    return {geometry::ToSE3(se3), intrinsics_to_optimize, summary.final_cost};
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
