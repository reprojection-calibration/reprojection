#include "optimization/nonlinear_refinement.hpp"

#include <ceres/ceres.h>

#include "eigen_utilities/camera.hpp"
#include "geometry/lie.hpp"
#include "pinhole_cost_function.hpp"

namespace reprojection::optimization {

// TODO(Jack): Increase consistency of the use of SE3 or se3 - we really only introduced the se3 in the general source
// code so that we could test pose values easily. Unless we are in the core optimization logic or testing we should be
// using SE3. Or at least that is my idea right now :)
// TODO(Jack): A function that converts from the matrix and array representation of K easily
// TODO(Jack): Would it help if we applied normalization?
std::tuple<Eigen::Isometry3d, Array4d> NonlinearRefinement(Eigen::MatrixX2d const& pixels,
                                                           Eigen::MatrixX3d const& points,
                                                           Eigen::Isometry3d const& initial_pose,
                                                           Eigen::Array4d const& initial_pinhole_intrinsics) {
    Eigen::Vector<double, 6> pose_to_optimize{geometry::Log(initial_pose)};
    Eigen::Array<double, 4, 1> pinhole_intrinsics_to_optimize{initial_pinhole_intrinsics};

    ceres::Problem problem;
    for (Eigen::Index i{0}; i < pixels.rows(); ++i) {
        ceres::CostFunction* const cost_function{PinholeCostFunction::Create(pixels.row(i), points.row(i))};
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
