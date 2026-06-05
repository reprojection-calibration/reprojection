#include "optimization/angular_velocity_alignment.hpp"

#include <ranges>

#include "cost_functions/rigid_body_angular_velocity.hpp"

namespace reprojection::optimization {

std::pair<Array3d, CeresState> AngularVelocityAlignment(VelocityMeasurements const& omega_imu,
                                                        spline::Se3Spline const& spline) {
    CeresState ceres_state{ceres::TAKE_OWNERSHIP, ceres::DENSE_SCHUR};
    ceres::Problem problem{ceres_state.problem_options};

    Array6d tf_imu_co{0, 0, 0, 0, 0, 0};
    // NOTE(Jack): We have to make a copy here even though we are not optimizing the spline (i.e. we set all spline
    // parameter blocks constant below), because the ceres AddResidualBlock() method requires the parameters are not
    // const.
    // TODO(Jack): Think of a better name than so3_spline_x?
    spline::Se3Spline spline_x{spline};
    for (auto const timestamp_ns : omega_imu | std::views::keys) {
        auto const normalized_position{
            spline_x.GetTimeHandler().SplinePosition(timestamp_ns, spline_x.ControlPoints().cols())};
        if (not normalized_position) {
            continue;  // LCOV_EXCL_LINE
        }
        auto const [u_i, i]{normalized_position.value()};

        ceres::CostFunction* const cost_function{cost_functions::RigidBodyAngularVelocity::Create(
            omega_imu.at(timestamp_ns).velocity, u_i, spline_x.GetTimeHandler().delta_t_ns_)};

        problem.AddResidualBlock(
            cost_function, nullptr, tf_imu_co.data(), spline_x.MutableControlPoints().col(i).data(),
            spline_x.MutableControlPoints().col(i + 1).data(), spline_x.MutableControlPoints().col(i + 2).data(),
            spline_x.MutableControlPoints().col(i + 3).data());

        // NOTE(Jack): We only want to initialize the extrinsic orientation between the imu and camera therefore we set
        // the control points constant.
        for (int j{0}; j < 4; ++j) {
            problem.SetParameterBlockConstant(spline_x.MutableControlPoints().col(i + j).data());
        }
    }

    ceres::Solve(ceres_state.solver_options, &problem, &ceres_state.solver_summary);

    return {tf_imu_co.topRows<3>(), ceres_state};
}

}  // namespace  reprojection::optimization
