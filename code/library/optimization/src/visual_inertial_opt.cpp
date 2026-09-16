#include "optimization/visual_inertial_opt.hpp"

#include <ceres/loss_function.h>

#include <ranges>

#include "cost_functions/reprojection_error_spline.hpp"
#include "cost_functions/rigid_body_angular_velocity.hpp"
#include "cost_functions/rigid_body_linear_acceleration.hpp"
#include "cost_functions/spline_energy.hpp"
#include "optimization/bundle_adjustment.hpp"
#include "spline/spline_init.hpp"

namespace reprojection::optimization {

std::pair<bundle_adjustment::VisualInertial::Result, CeresState> VisualInertialOpt(
    bundle_adjustment::VisualInertial::Problem const& problem, int num_threads) {
    bundle_adjustment::VisualInertial::Result result{problem};
    auto& spline{result.rig.spline};

    // TODO(Jack): What is the correct linear solver?
    CeresState ceres_state{ceres::TAKE_OWNERSHIP, ceres::SPARSE_NORMAL_CHOLESKY, num_threads};
    ceres::Problem ceres_problem{ceres_state.problem_options};

    // Imu residuals
    for (auto const timestamp_ns : problem.imu_data | std::views::keys) {
        auto const normalized_position{
            spline.GetTimeHandler().SplinePosition(timestamp_ns, spline.ControlPoints().cols())};
        if (not normalized_position.has_value()) {
            continue;  // LCOV_EXCL_LINE
        }
        auto const [u_i, i]{normalized_position.value()};

        auto const& imu_data_i{problem.imu_data.at(timestamp_ns)};

        // WARN(Jack): We pass in the pointer to the full tf and control points but the angular velocity cost function
        // only uses the top three rows of all. Is there a better design?
        ceres::CostFunction* const gyroscope_cost_function{cost_functions::RigidBodyAngularVelocity::Create(
            imu_data_i.angular_velocity, u_i, spline.GetTimeHandler().delta_t_ns_)};
        ceres_problem.AddResidualBlock(gyroscope_cost_function, nullptr,          //
                                       result.inertial_state.se3_imu_rig.data(),  //
                                       spline.ControlPoint(i),                    //
                                       spline.ControlPoint(i + 1),                //
                                       spline.ControlPoint(i + 2),                //
                                       spline.ControlPoint(i + 3));

        ceres::CostFunction* const accelerometer_cost_function{cost_functions::RigidBodyLinearAcceleration::Create(
            imu_data_i.linear_acceleration, u_i, spline.GetTimeHandler().delta_t_ns_)};
        ceres_problem.AddResidualBlock(accelerometer_cost_function, nullptr,      //
                                       result.inertial_state.se3_imu_rig.data(),  //
                                       result.inertial_state.gravity_w.data(),    //
                                       spline.ControlPoint(i),                    //
                                       spline.ControlPoint(i + 1),                //
                                       spline.ControlPoint(i + 2),                //
                                       spline.ControlPoint(i + 3));
    }

    // Reprojection residuals
    for (auto const& [camera_id, sample_timestamp_ns, _, bundle] : problem.observations) {
        auto const normalized_position{
            spline.GetTimeHandler().SplinePosition(sample_timestamp_ns, spline.ControlPoints().cols())};
        if (not normalized_position.has_value()) {
            continue;
        }
        auto const [u_i, i]{normalized_position.value()};

        auto const& [camera_info, _1,
                     camera_options]{problem.cameras.at(camera_id)};  // cppcheck-suppress ignoredReturnValue
        auto& camera_state{result.camera_states.at(camera_id)};

        auto const& [pixels, points]{bundle};
        for (Eigen::Index j{0}; j < pixels.rows(); ++j) {
            ceres::CostFunction* const cost_function{
                cost_functions::Create(camera_info.camera_model, camera_info.bounds, pixels.row(j), points.row(j), u_i,
                                       spline.GetTimeHandler().delta_t_ns_)};

            // TODO(Jack): Should we also use robust loss here like we use for the stand alone bundle adjustment?
            ceres_problem.AddResidualBlock(cost_function, nullptr,               //
                                           camera_state.intrinsic.value.data(),  //
                                           camera_state.extrinsic.data(),        //
                                           spline.ControlPoint(i),               //
                                           spline.ControlPoint(i + 1),           //
                                           spline.ControlPoint(i + 2),           //
                                           spline.ControlPoint(i + 3));
        }

        if (not camera_options.optimize_intrinsic) {
            ceres_problem.SetParameterBlockConstant(camera_state.intrinsic.value.data());
        }
        if (not camera_options.optimize_extrinsic) {
            ceres_problem.SetParameterBlockConstant(camera_state.extrinsic.data());
        }
    }

    // Smoothness/minimum energy constraint
    for (int i{0}; i < spline.Size() - 3; ++i) {
        ceres::CostFunction* const cost_function{cost_functions::SplineEnergy::Create(1)};
        ceres_problem.AddResidualBlock(cost_function, nullptr,      //
                                       spline.ControlPoint(i),      //
                                       spline.ControlPoint(i + 1),  //
                                       spline.ControlPoint(i + 2),  //
                                       spline.ControlPoint(i + 3));
    }

    ceres::Solve(ceres_state.solver_options, &ceres_problem, &ceres_state.solver_summary);

    return {result, ceres_state};
}

// NOTE(Jack): We build the canonical bundle adjustment problem here ONLY so we can use the standard bundle adjustment
// reprojection error calculation.
// NOTE(Jack): There is something nice about using the same exact logic from the optimization (i.e. cost functions) when
// calculating an optimization's residuals. That being said we eliminated a lot of code duplication by just using the
// spline.Evaluate() interface and filling out the canonical bundle adjustment problem here.
bundle_adjustment::Discrete::Problem ToBaProblem(bundle_adjustment::VisualInertial::Problem const& problem) {
    // NOTE(Jack): Something actually really important is happening here that is a result of the continuous spline
    // representation. And that is that observations which maybe did not have a matching discrete frame, and therefore
    // would have been ignored, can be handled here because the spline can interpolate the pose for any "on spline"
    // time. Note that this will be the rig frame pose and you will need the extrinsic to get it into the actualy camera
    // frame the target observation comes from.
    Frames frames;
    for (auto const& [_, sample_timestamp_ns, _1, target] : problem.observations) {
        if (auto const tf_w_co{problem.rig.spline.Evaluate(sample_timestamp_ns, spline::DerivativeOrder::Null)}) {
            // NOTE(Jack): Inverse the spline pose to put it into the classic bundle adjustment friendly convention of
            // transforming points from the world into the camera.
            Array6d const tf_co_w{geometry::InverseTransform<double>(*tf_w_co)};
            // NOTE(Jack): map.insert() will NOT update the value if one already exists. So if you have exact matching
            // timestamps (i.e. a perfectly synced stereo camera) then only the first spline interpolation will be
            // added. That is fine because at any one timestamp the rig pose has to be the same!
            frames.insert({sample_timestamp_ns, {tf_co_w}});
        }
    }

    return bundle_adjustment::Discrete::Problem{problem.rig.asset_id, frames, problem.cameras, problem.observations};
}

ImuErrors EvaluateImuError(ImuSamples const& imu_data, Extrinsic const& extrinsic, Vector3d const& gravity,
                           spline::Se3Spline const& spline_w_co) {
    ImuErrors imu_residuals;

    for (auto const timestamp_ns : imu_data | std::views::keys) {
        // TODO(Jack): This logic is now repeated several times... we are missing the point I think. How to fix!?
        auto const normalized_position{spline_w_co.GetTimeHandler().SplinePosition(timestamp_ns, spline_w_co.Size())};
        if (not normalized_position.has_value()) {
            continue;  // LCOV_EXCL_LINE
        }
        auto const [u_i, i]{normalized_position.value()};

        std::vector<double const*> parameter_blocks;
        parameter_blocks.push_back(extrinsic.se3_a_b.data());
        for (int j{0}; j < 4; ++j) {
            parameter_blocks.push_back(spline_w_co.ControlPoint(i + j));
        }
        ceres::CostFunction const* const cost_function_1{cost_functions::RigidBodyAngularVelocity::Create(
            imu_data.at(timestamp_ns).angular_velocity, u_i, spline_w_co.GetTimeHandler().delta_t_ns_)};

        // WARN(Jack): If we ever decide to remove the gravity residual then we need to remember to change this back to
        // length 6 and also remove the .segment() logic below!
        Array7d residual_i;
        cost_function_1->Evaluate(parameter_blocks.data(), residual_i.topRows<3>().data(), nullptr);

        parameter_blocks.insert(std::cbegin(parameter_blocks) + 1, gravity.data());
        ceres::CostFunction const* const cost_function_2{cost_functions::RigidBodyLinearAcceleration::Create(
            imu_data.at(timestamp_ns).linear_acceleration, u_i, spline_w_co.GetTimeHandler().delta_t_ns_)};

        cost_function_2->Evaluate(parameter_blocks.data(), residual_i.bottomRows<4>().data(), nullptr);

        // TODO(Jack): Should we use a smart pointer instead?
        delete cost_function_1;
        delete cost_function_2;

        imu_residuals.insert({timestamp_ns, {residual_i.topRows<3>(), residual_i.segment(3, 3)}});
    }

    return imu_residuals;
}  // LCOV_EXCL_LINE

}  // namespace  reprojection::optimization
