#include "nonlinear_refinement.hpp"

#include "pose_utilities.hpp"

namespace reprojection_calibration::pnp {

// TODO(Jack): Increase consistency of the use of SE3 or se3 - we really only introduced the se3 in the general source
// code so that we could test pose values easily. Unless we are in the core optimization logic or testing we should be
// using SE3. Or at least that is my idea right now :)
// TODO(Jack): A function that converts from the matrix and array representation of K easily
// TODO(Jack): Would it help if we applied normalization?
std::tuple<Eigen::Isometry3d, Eigen::Matrix3d> NonlinearRefinement(Eigen::MatrixX2d const& pixels,
                                                                   Eigen::MatrixX3d const& points,
                                                                   Eigen::Isometry3d const& initial_pose,
                                                                   Eigen::Matrix3d const& initial_K) {
    Se3 pose_to_optimize{ToSe3(initial_pose)};
    Eigen::Array<double, 4, 1> pinhole_intrinsics_to_optimize{FromK(initial_K)};

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

    return {FromSe3(pose_to_optimize), ToK(pinhole_intrinsics_to_optimize)};
}

// TODO(Jack): This might belong in some other file with camera geometry specific helper functions
Eigen::Matrix3d ToK(Eigen::Array<double, 4, 1> const& array) {
    Eigen::Matrix3d K{Eigen::Matrix3d::Identity()};
    K(0, 0) = array[0];
    K(1, 1) = array[1];
    K(0, 2) = array[2];
    K(1, 2) = array[3];

    return K;
};

Eigen::Array<double, 4, 1> FromK(Eigen::Matrix3d const& matrix) {
    return {matrix(0, 0), matrix(1, 1), matrix(0, 2), matrix(1, 2)};
};

}  // namespace reprojection_calibration::pnp
