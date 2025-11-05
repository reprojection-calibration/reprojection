#include "optimization/nonlinear_refinement.hpp"

#include <ceres/ceres.h>

#include <algorithm>

#include "geometry/lie.hpp"
#include "projection_cost_function.hpp"

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

    std::vector<Isometry3d> SE3;
    std::transform(std::cbegin(se3), std::cend(se3), std::back_inserter(SE3),
                   [](Array6d const& se3_i) { return geometry::Exp(Vector6d{se3_i}); });

    return {SE3, intrinsics_to_optimize, summary.final_cost};
}

}  // namespace  reprojection::optimization
