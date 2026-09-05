#include "optimization/extrinsic_optimization.hpp"

#include <ceres/loss_function.h>

#include <ranges>

#include "cost_functions/reprojection_error_spline.hpp"
#include "cost_functions/rigid_body_angular_velocity.hpp"
#include "cost_functions/rigid_body_linear_acceleration.hpp"
#include "cost_functions/spline_energy.hpp"
#include "optimization/bundle_adjustment.hpp"
#include "spline/spline_initialization.hpp"

namespace reprojection::optimization {

using Ba = BundleAdjustment;

std::tuple<spline::Se3Spline, Extrinsic, Vector3d> ExtrinsicOptimization(
    ImuSamples const& imu_data, spline::Se3Spline const& initial_spline, Extrinsic const& initial_extrinsic,
    Vector3d const& initial_gravity, CameraInfo const& sensor, TargetSamples const& targets, Intrinsic const& intrinsic,
    int const num_threads) {
    // TODO(Jack): What is the correct linear solver?
    CeresState ceres_state{ceres::TAKE_OWNERSHIP, ceres::SPARSE_NORMAL_CHOLESKY};
    ceres_state.solver_options.num_threads = num_threads;
    ceres::Problem problem{ceres_state.problem_options};

    spline::Se3Spline optimized_spline{initial_spline};
    Extrinsic optimized_extrinsic{initial_extrinsic};
    Vector3d optimized_gravity{initial_gravity};

    // Imu residuals
    for (auto const timestamp_ns : imu_data | std::views::keys) {
        auto const normalized_position{
            optimized_spline.GetTimeHandler().SplinePosition(timestamp_ns, optimized_spline.ControlPoints().cols())};
        if (not normalized_position.has_value()) {
            continue;  // LCOV_EXCL_LINE
        }
        auto const [u_i, i]{normalized_position.value()};

        // WARN(Jack): We pass in the pointer to the full tf and control points but the angular velocity cost function
        // only uses the top three rows of all. Is there a better design?
        ceres::CostFunction* const gyroscope_cost_function{cost_functions::RigidBodyAngularVelocity::Create(
            imu_data.at(timestamp_ns).angular_velocity, u_i, optimized_spline.GetTimeHandler().delta_t_ns_)};
        problem.AddResidualBlock(gyroscope_cost_function, nullptr, optimized_extrinsic.se3_a_b.data(),
                                 optimized_spline.MutableControlPoints().col(i).data(),
                                 optimized_spline.MutableControlPoints().col(i + 1).data(),
                                 optimized_spline.MutableControlPoints().col(i + 2).data(),
                                 optimized_spline.MutableControlPoints().col(i + 3).data());

        ceres::CostFunction* const accelerometer_cost_function{cost_functions::RigidBodyLinearAcceleration::Create(
            imu_data.at(timestamp_ns).linear_acceleration, u_i, optimized_spline.GetTimeHandler().delta_t_ns_)};
        problem.AddResidualBlock(accelerometer_cost_function, nullptr, optimized_extrinsic.se3_a_b.data(),
                                 optimized_gravity.data(), optimized_spline.MutableControlPoints().col(i).data(),
                                 optimized_spline.MutableControlPoints().col(i + 1).data(),
                                 optimized_spline.MutableControlPoints().col(i + 2).data(),
                                 optimized_spline.MutableControlPoints().col(i + 3).data());
    }

    // Reprojection residuals
    Intrinsic intrinsic_x{intrinsic};
    for (auto const timestamp_ns : targets | std::views::keys) {
        auto const normalized_position{
            optimized_spline.GetTimeHandler().SplinePosition(timestamp_ns, optimized_spline.ControlPoints().cols())};
        if (not normalized_position.has_value()) {
            continue;  // LCOV_EXCL_LINE
        }
        auto const [u_i, i]{normalized_position.value()};

        // TODO(Jack): Copy and pasted from reprojectiom error below
        auto const& [pixels, points]{targets.at(timestamp_ns).bundle};
        for (Eigen::Index j{0}; j < pixels.rows(); ++j) {
            ceres::CostFunction* const cost_function{
                cost_functions::Create(sensor.camera_model, sensor.bounds, pixels.row(j), points.row(j), u_i,
                                       optimized_spline.GetTimeHandler().delta_t_ns_)};
            // TODO(Jack): Should we also use robust loss here like we use for the stand alone bundle adjustment?
            problem.AddResidualBlock(cost_function, nullptr, intrinsic_x.value.data(),
                                     optimized_spline.MutableControlPoints().col(i).data(),
                                     optimized_spline.MutableControlPoints().col(i + 1).data(),
                                     optimized_spline.MutableControlPoints().col(i + 2).data(),
                                     optimized_spline.MutableControlPoints().col(i + 3).data());
        }
    }

    // Smoothness/minimum energy constraint
    for (int i{0}; i < optimized_spline.Size() - 3; ++i) {
        ceres::CostFunction* const cost_function{cost_functions::SplineEnergy::Create(1)};
        problem.AddResidualBlock(cost_function, nullptr, optimized_spline.MutableControlPoints().col(i).data(),
                                 optimized_spline.MutableControlPoints().col(i + 1).data(),
                                 optimized_spline.MutableControlPoints().col(i + 2).data(),
                                 optimized_spline.MutableControlPoints().col(i + 3).data());
    }

    // This was already solved for in the bundle adjustment step, therefore I do not think there is a good reason to
    // further optimize it here.
    problem.SetParameterBlockConstant(intrinsic_x.value.data());
    ceres::Solve(ceres_state.solver_options, &problem, &ceres_state.solver_summary);

    return {optimized_spline, optimized_extrinsic, optimized_gravity};
}

// NOTE(Jack): We build the canonical bundle adjustment problem here ONLY so we can use the standard bundle adjustment
// reprojection error calculation. We never optimized this problem from the spline data directly.
// NOTE(Jack): I think there is something nice about using the same exact logic from the optimization (i.e. cost
// functions) when calculating an optimization's residuals. That being said we eliminated a lot of code duplication by
// just using the spline.Evaluate() interface and filling out the canonical bundle adjustment problem here.
Ba::Problem SingleSplineCamProblem(CameraInfo const& camera_info, Intrinsic const& intrinsic,
                                   TargetSamples const& targets, spline::Se3Spline const& spline_w_co,
                                   AssetId const camera_id) {
    // For a single camera problem we do not consider the rig-camera extrinsic and set those to constant identity.
    Ba::Camera const camera{camera_info, Ba::CameraState{intrinsic, Array6d::Zero()}, {}};

    // TODO(Jack): Should we do any check that the frame times match all the target times? Or is that something we need
    // to just check once when we actually construct the problem?
    Frames frames;
    std::vector<Ba::Observation> observations;
    for (auto const& [timestamp_ns, target] : targets) {
        if (auto const tf_w_co{spline_w_co.Evaluate(timestamp_ns, spline::DerivativeOrder::Null)}) {
            // Inverse the spline pose to put it into the classic bundle adjustment friendly convention of transforming
            // points from the world into the camera.
            Array6d const tf_co_w{geometry::InverseTransform<double>(*tf_w_co)};
            frames.insert({timestamp_ns, {tf_co_w}});
        }

        // NOTE(Jack): For a single cam the sample and frame timestamps are by definition the same! Any sensor it
        // automatically self synchronized.
        observations.push_back({camera_id, timestamp_ns, timestamp_ns, target.bundle});
    }

    return {{{camera_id, camera}}, frames, observations};
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
            parameter_blocks.push_back(spline_w_co.ControlPoints().col(i + j).data());
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
