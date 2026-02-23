#include "optimization/angular_velocity_alignment.hpp"

#include <ranges>

#include "geometry/lie.hpp"

#include "angular_velocity_cost_function.hpp"

namespace reprojection::optimization {

// TODO(Jack): This function is dependent on the data being synchronized - how can we assert or easily tell the user
//  when there is a problem here? For now we will iterate over omega_co because this should only be as large as
//  omega_imu but cannot be larger because it is sampled from the interpolated spline using the timestamps from the imu
//  data. Therefore it might be smaller, but cannot include a timestamp which is not also in omega_imu.
std::tuple<Matrix3d, CeresState> AngularVelocityAlignment(VelocityMeasurements const& omega_a,
                                                          VelocityMeasurements const& omega_b) {
    CeresState ceres_state{ceres::TAKE_OWNERSHIP, ceres::DENSE_SCHUR};
    ceres::Problem problem{ceres_state.problem_options};

    Array3d aa_a_b{0, 0, 0};
    for (auto const timestamp_ns : omega_a | std::views::keys) {
        ceres::CostFunction* const cost_function{
            AngularVelocityCostFunction::Create(omega_a.at(timestamp_ns).velocity, omega_b.at(timestamp_ns).velocity)};

        problem.AddResidualBlock(cost_function, nullptr, aa_a_b.data());
    }

    ceres::Solve(ceres_state.solver_options, &problem, &ceres_state.solver_summary);

    return {geometry::Exp<double>(aa_a_b), ceres_state};
}

}  // namespace  reprojection::optimization
