#include "optimization/angular_velocity_alignment.hpp"

#include <ranges>
#include <thread>

#include "cost_functions/rigid_body_angular_velocity.hpp"

namespace reprojection::optimization {

// NOTE(Jack): The angular velocity initialization actually only uses the so3 rotation component of the spline. But our
// rigid body angular velocity cost function requires the full control points because ceres does not allow parameter
// aliasing (see comment at RigidBodyAngularVelocity::Create()).
std::pair<Array3d, CeresState> AngularVelocityAlignment(VelocityMeasurements const& omega_imu, spline::Se3Spline spline,
                                                        int const num_threads) {
    // TODO(Jack): We need a better more uniform way of parameterizing the ceres optimizations.
    CeresState ceres_state{ceres::TAKE_OWNERSHIP, ceres::DENSE_SCHUR};
    ceres_state.solver_options.num_threads = num_threads;
    ceres::Problem problem{ceres_state.problem_options};

    Array6d tf_imu_co{0, 0, 0, 0, 0, 0};
    for (auto const timestamp_ns : omega_imu | std::views::keys) {
        auto const normalized_position{spline.GetTimeHandler().SplinePosition(timestamp_ns, spline.Size())};
        if (not normalized_position) {
            continue;  // LCOV_EXCL_LINE
        }
        auto const [u_i, i]{normalized_position.value()};

        ceres::CostFunction* const cost_function{cost_functions::RigidBodyAngularVelocity::Create(
            omega_imu.at(timestamp_ns).velocity, u_i, spline.GetTimeHandler().delta_t_ns_)};

        problem.AddResidualBlock(cost_function, nullptr, tf_imu_co.data(),     //
                                 spline.MutableControlPoints().col(i).data(),  //
                                 spline.MutableControlPoints().col(i + 1).data(),
                                 spline.MutableControlPoints().col(i + 2).data(),
                                 spline.MutableControlPoints().col(i + 3).data());

        // NOTE(Jack): We only want to initialize the extrinsic orientation between the imu and camera therefore we set
        // the control points constant.
        for (int j{0}; j < 4; ++j) {
            problem.SetParameterBlockConstant(spline.MutableControlPoints().col(i + j).data());
        }
    }

    // TODO(Jack): Copy and pasted!
    unsigned int const hw_threads{std::thread::hardware_concurrency()};
    ceres_state.solver_options.num_threads = hw_threads > 1 ? hw_threads - 1 : 1;

    ceres::Solve(ceres_state.solver_options, &problem, &ceres_state.solver_summary);

    return {tf_imu_co.topRows<3>(), ceres_state};
}

}  // namespace  reprojection::optimization
