#include "optimization/nonlinear_refinement.hpp"

#include <ceres/ceres.h>

#include "eigen_utilities/camera.hpp"
#include "geometry/lie.hpp"
#include "projection_cost_function.hpp"

namespace reprojection::optimization {

// NOTE(Jack): Hardcoded only for pinhole
std::tuple<Isometry3d, Array4d> NonlinearRefinement(MatrixX2d const& pixels, MatrixX3d const& points,
                                                    Isometry3d const& initial_pose,
                                                    Array4d const& initial_pinhole_intrinsics) {
    Eigen::Vector<double, 6> pose_to_optimize{geometry::Log(initial_pose)};
    Array4d pinhole_intrinsics_to_optimize{initial_pinhole_intrinsics};

    ceres::Problem problem;
    for (Eigen::Index i{0}; i < pixels.rows(); ++i) {
        ceres::CostFunction* const cost_function{Create(CameraModel::Pinhole, pixels.row(i), points.row(i))};
        problem.AddResidualBlock(cost_function, nullptr, pinhole_intrinsics_to_optimize.data(),
                                 pose_to_optimize.data());
    }

    // TODO(Jack): Law of useful return states that we should probably be returning this diagnostic information so that
    // people can diagnose failures.
    // TODO(Jack): Tune best optimizer options for the problem at hand
    ceres::Solver::Options options;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    return {geometry::Exp(pose_to_optimize), pinhole_intrinsics_to_optimize};
}

}  // namespace  reprojection::optimization
