#include "optimization/nonlinear_refinement.hpp"

#include <ceres/ceres.h>

#include "geometry/lie.hpp"
#include "projection_cost_function.hpp"

namespace reprojection::optimization {

// TODO(Jack): Should we have some assertions which force that the frames satisfy some basic properties like there is a
// matching number of everything?
std::tuple<std::vector<Isometry3d>, ArrayXd> NonlinearRefinement(std::vector<Frame> const& frames,
                                                                 CameraModel const& camera_type,
                                                                 ArrayXd const& intrinsics) {
    assert(static_cast<int>(camera_type) == intrinsics.rows());  // This is not a real error handling stategy!

    std::vector<Array6d> poses_to_optimize;
    poses_to_optimize.reserve(std::size(frames));
    ArrayXd intrinsics_to_optimize{intrinsics};  // Same intrinsics for ALL poses - mono camera constraint

    ceres::Problem problem;
    for (size_t i{0}; i < std::size(frames); ++i) {
        MatrixX2d const& pixels_i{frames[i].bundle.pixels};
        MatrixX3d const& points_i{frames[i].bundle.points};
        poses_to_optimize.push_back(geometry::Log(frames[i].pose));

        for (Eigen::Index j{0}; j < pixels_i.rows(); ++j) {
            ceres::CostFunction* const cost_function{Create(camera_type, pixels_i.row(j), points_i.row(j))};

            problem.AddResidualBlock(cost_function, nullptr, intrinsics_to_optimize.data(),
                                     poses_to_optimize[i].data());
        }
    }

    // TODO(Jack): Law of useful return states that we should probably be returning this diagnostic information so that
    // people can diagnose failures.
    // TODO(Jack): Tune best optimizer options for the problem at hand
    ceres::Solver::Options options;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // TODO(Jack): If we use this logic in multiple places for conversions than make sure to put it into a helper
    // method!
    std::vector<Isometry3d> poses_to_return;
    poses_to_return.reserve(std::size(poses_to_optimize));
    for (Array6d const& optimized_pose : poses_to_optimize) {
        // TODO(Jack): Remove temp! Required because of overloading problems with Array6d.
        Vector6d const temp{optimized_pose};
        poses_to_return.push_back(geometry::Exp(temp));
    }

    return {poses_to_return, intrinsics_to_optimize};
}

}  // namespace  reprojection::optimization
