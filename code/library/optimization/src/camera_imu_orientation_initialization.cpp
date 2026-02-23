#include "optimization/camera_imu_orientation_initialization.hpp"

#include <ranges>

#include "geometry/lie.hpp"
#include "optimization/ceres_state.hpp"

#include "rotational_velocity_cost_function.hpp"

namespace reprojection::optimization {

// TODO(Jack): This function is dependent on the data being synchronized - how can we assert or easily tell the user
//  when there is a problem here? For now we will iterate over omega_co because this should only be as large as
//  omega_imu but cannot be larger because it is sampled from the interpolated spline using the timestamps from the imu
//  data. Therefore it might be smaller, but cannot include a timestamp which is not also in omega_imu.
// NOTE(Jack): This function makes the approximation that the centers of rotation (i.e. the camera and imu sensors) are
// located at the same position. That is why only the rotation here is being optimized. It is an approximation so that
// we can initialize the full extrinsic optimization later.
std::tuple<Matrix3d, CeresState> InitializeCameraImuOrientation(VelocityMeasurements const& omega_co,
                                                                VelocityMeasurements const& omega_imu) {
    CeresState ceres_state{ceres::TAKE_OWNERSHIP, ceres::DENSE_SCHUR};
    ceres::Problem problem{ceres_state.problem_options};

    Array3d aa_co_imu{0, 0, 0};
    for (auto const timestamp_ns : omega_co | std::views::keys) {
        ceres::CostFunction* const cost_function{RotationalVelocityCostFunction::Create(
            omega_co.at(timestamp_ns).velocity, omega_imu.at(timestamp_ns).velocity)};

        problem.AddResidualBlock(cost_function, nullptr, aa_co_imu.data());
    }

    ceres::Solve(ceres_state.solver_options, &problem, &ceres_state.solver_summary);

    return {geometry::Exp<double>(aa_co_imu), ceres_state};
}

}  // namespace  reprojection::optimization
