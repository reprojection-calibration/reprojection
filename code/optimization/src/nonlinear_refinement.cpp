#include "optimization/nonlinear_refinement.hpp"

#include "geometry/lie.hpp"
#include "projection_cost_function.hpp"
#include "r3_spline_cost_function.hpp"
#include "spline/r3_spline.hpp"

namespace reprojection::optimization {

// TODO(Jack): Should we have some assertions which force that the frames satisfy some basic properties like there is a
// matching number of everything?
std::tuple<std::vector<Isometry3d>, ArrayXd, double> NonlinearRefinement(std::vector<Frame> const& frames,
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
R3SplineProblemHandler::R3SplineProblemHandler(spline::R3SplineState const& spline) : spline_{spline} {}

// NOTE(Jack): We will keep this as no discard because I want to force the user to responsibly handle invalid
// conditions when adding constraints. This is one distinction from the camera bundle adjustment case, that here we have
// an explicit condition that determines what is valid and what is not, which requires more fine grained control from
// the user side.
[[nodiscard]] bool R3SplineProblemHandler::AddConstraint(R3Measurement const& constraint) {
    auto const normalized_position{
        spline_.time_handler.SplinePosition(constraint.t_ns, std::size(spline_.control_points))};
    if (not normalized_position.has_value()) {
        return false;
    }
    auto const [u_i, i]{normalized_position.value()};

    ceres::CostFunction* const cost_function{optimization::CreateR3SplineCostFunction(
        constraint.type, constraint.r3, u_i, spline_.time_handler.delta_t_ns_)};

    problem_.AddResidualBlock(cost_function, nullptr, spline_.control_points[i].data(),
                              spline_.control_points[i + 1].data(), spline_.control_points[i + 2].data(),
                              spline_.control_points[i + 3].data());

    return true;
}

// TODO(Jack): There is no protection which would prevent the user from calling this on an invalid problem (ex. they
// forgot to add any data). We need to codify the real long term usage strategy here, this is not the final answer!
ceres::Solver::Summary R3SplineProblemHandler::Solve() {
    ceres::Solver::Options options;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem_, &summary);

    return summary;
}

spline::R3SplineState R3SplineProblemHandler::GetSpline() const { return spline_; }

}  // namespace  reprojection::optimization
