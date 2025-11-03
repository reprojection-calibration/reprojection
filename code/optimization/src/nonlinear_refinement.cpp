#include "optimization/nonlinear_refinement.hpp"

#include <ceres/ceres.h>

#include "eigen_utilities/camera.hpp"
#include "geometry/lie.hpp"
#include "projection_cost_function.hpp"

namespace reprojection::optimization {

// NOTE(Jack): Hardcoded only for pinhole
std::tuple<std::vector<Isometry3d>, ArrayXd> NonlinearRefinement(std::vector<MatrixX2d> const& pixels,
                                                                 std::vector<MatrixX3d> const& points,
                                                                 std::vector<Isometry3d> const& poses,
                                                                 CameraModel const& camera_type,
                                                                 ArrayXd const& intrinsics) {
    // NOTE(Jack): I think the fact that ALL the input parameters have some relation to another one indicates that these
    // could actually be a type itself! THe benefit of that is that we could put conditions on the type to make sure the
    // invariants are met, instead of having to check them here and at n other locations.
    assert(std::size(pixels) == std::size(points));
    assert(std::size(pixels) == std::size(poses));
    assert(static_cast<int>(camera_type) == intrinsics.rows());

    std::vector<Array6d> poses_to_optimize;
    poses_to_optimize.reserve(std::size(poses));
    ArrayXd intrinsics_to_optimize{intrinsics};  // Same intrinsics for ALL poses - mono camera constraint

    ceres::Problem problem;
    for (size_t i{0}; i < std::size(pixels); ++i) {
        MatrixX2d const& pixels_i{pixels[i]};
        MatrixX3d const& points_i{points[i]};
        poses_to_optimize.push_back(geometry::Log(poses[i]));

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
        Eigen::Vector<double, 6> const temp{optimized_pose};
        poses_to_return.push_back(geometry::Exp(temp));
    }

    return {poses_to_return, intrinsics_to_optimize};
}

std::tuple<Isometry3d, ArrayXd> NonlinearRefinement(MatrixX2d const& pixels, MatrixX3d const& points,
                                                    Isometry3d const& poses, CameraModel const& camera_type,
                                                    ArrayXd const& intrinsics) {
    std::vector<MatrixX2d> const pixels_vector{pixels};
    std::vector<MatrixX3d> const points_vector{points};
    std::vector<Isometry3d> const poses_vector{poses};

    auto const [optimized_poses, optimized_intrinsics]{
        NonlinearRefinement(pixels_vector, points_vector, poses_vector, camera_type, intrinsics)};

    return {optimized_poses[0], optimized_intrinsics};
}

}  // namespace  reprojection::optimization
