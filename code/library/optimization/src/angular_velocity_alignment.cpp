#include "optimization/angular_velocity_alignment.hpp"

#include <ranges>

#include "cost_functions/rigid_body_angular_velocity.hpp"

namespace reprojection::optimization {

std::pair<Array3d, CeresState> AngularVelocityAlignment(VelocityMeasurements const& omega_imu,
                                                        spline::CubicBSplineC3 so3_spline) {
    CeresState ceres_state{ceres::TAKE_OWNERSHIP, ceres::DENSE_SCHUR};
    ceres::Problem problem{ceres_state.problem_options};

    Array3d aa_a_b{0, 0, 0};
    for (auto const timestamp_ns : omega_imu | std::views::keys) {
        auto const normalized_position{so3_spline.GetTimeHandler().SplinePosition(timestamp_ns, so3_spline.Size())};
        if (not normalized_position) {
            continue;  // LCOV_EXCL_LINE
        }
        auto const [u_i, i]{normalized_position.value()};

        ceres::CostFunction* const cost_function{cost_functions::RigidBodyAngularVelocity::Create(
            omega_imu.at(timestamp_ns).velocity, u_i, so3_spline.GetTimeHandler().delta_t_ns_)};

        problem.AddResidualBlock(cost_function, nullptr, aa_a_b.data(), so3_spline.MutableControlPoints().col(i).data(),
                                 so3_spline.MutableControlPoints().col(i + 1).data(),
                                 so3_spline.MutableControlPoints().col(i + 2).data(),
                                 so3_spline.MutableControlPoints().col(i + 3).data());

        // NOTE(Jack): We only want to initialize the extrinsic orientation between the imu and camera therefore we set
        // the control points constant.
        for (int j{0}; j < 4; ++j) {
            problem.SetParameterBlockConstant(so3_spline.MutableControlPoints().col(i + j).data());
        }
    }

    ceres::Solve(ceres_state.solver_options, &problem, &ceres_state.solver_summary);

    return {aa_a_b, ceres_state};
}

}  // namespace  reprojection::optimization
