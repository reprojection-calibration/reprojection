#include "optimization/extrinsic_optimization.hpp"

#include <ranges>

#include "cost_functions/reprojection_error_spline.hpp"
#include "cost_functions/rigid_body_angular_velocity.hpp"
#include "cost_functions/rigid_body_linear_acceleration.hpp"
#include "spline/spline_initialization.hpp"

namespace reprojection::optimization {

std::pair<spline::Se3Spline, ImuCamExtrinsic> ExtrinsicOptimization(
    ImuMeasurements const& imu_data, spline::Se3Spline const& initial_spline, ImuCamExtrinsic const& initial_extrinsic,
    CameraInfo const& sensor, CameraMeasurements const& targets, CameraState const& intrinsics) {
    // TODO(Jack): What is the correct linear solver?
    CeresState ceres_state{ceres::TAKE_OWNERSHIP, ceres::SPARSE_NORMAL_CHOLESKY};
    ceres::Problem problem{ceres_state.problem_options};

    spline::Se3Spline optimized_spline{initial_spline};
    ImuCamExtrinsic optimized_extrinsic{initial_extrinsic};

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
        problem.AddResidualBlock(gyroscope_cost_function, nullptr, optimized_extrinsic.tf.se3_a_b.data(),
                                 optimized_spline.MutableControlPoints().col(i).data(),
                                 optimized_spline.MutableControlPoints().col(i + 1).data(),
                                 optimized_spline.MutableControlPoints().col(i + 2).data(),
                                 optimized_spline.MutableControlPoints().col(i + 3).data());

        ceres::CostFunction* const accelerometer_cost_function{cost_functions::RigidBodyLinearAcceleration::Create(
            imu_data.at(timestamp_ns).linear_acceleration, u_i, optimized_spline.GetTimeHandler().delta_t_ns_)};
        problem.AddResidualBlock(accelerometer_cost_function, nullptr, optimized_extrinsic.tf.se3_a_b.data(),
                                 optimized_extrinsic.gravity.data(),
                                 optimized_spline.MutableControlPoints().col(i).data(),
                                 optimized_spline.MutableControlPoints().col(i + 1).data(),
                                 optimized_spline.MutableControlPoints().col(i + 2).data(),
                                 optimized_spline.MutableControlPoints().col(i + 3).data());
    }

    // Reprojection residuals
    CameraState intrinsics_x{intrinsics};
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
            problem.AddResidualBlock(cost_function, nullptr, intrinsics_x.intrinsics.data(),
                                     optimized_spline.MutableControlPoints().col(i).data(),
                                     optimized_spline.MutableControlPoints().col(i + 1).data(),
                                     optimized_spline.MutableControlPoints().col(i + 2).data(),
                                     optimized_spline.MutableControlPoints().col(i + 3).data());
        }
    }

    // This was already solved for in the bundle adjustment step, therefore I do not think there is a good reason to
    // further optimize it here.
    problem.SetParameterBlockConstant(intrinsics_x.intrinsics.data());

    ceres_state.solver_options.minimizer_progress_to_stdout = true;
    ceres::Solve(ceres_state.solver_options, &problem, &ceres_state.solver_summary);
    std::cout << ceres_state.solver_summary.FullReport() << std::endl;

    return {optimized_spline, optimized_extrinsic};
}

std::pair<Frames, ReprojectionErrors> ReprojectionErrorSpline(CameraInfo const& sensor,
                                                              CameraMeasurements const& targets,
                                                              CameraState const& camera_state,
                                                              spline::Se3Spline const& spline) {
    // TODO(Jack): We are calculating the reprojection errors for all targets that are on the interpolated spline. That
    //  means that even if there is no initial pose that we will have an evaluation. This means there can be no foreign
    //  key constraint. Do we need new tables for this?
    Frames poses;
    ReprojectionErrors residuals;
    for (auto const timestamp_ns : targets | std::views::keys) {
        auto const pose{spline.Evaluate(timestamp_ns, spline::DerivativeOrder::Null)};
        if (not pose) {
            continue;
        }
        poses.insert({timestamp_ns, {*pose}});

        auto const normalized_position{spline.GetTimeHandler().SplinePosition(timestamp_ns, spline.Size())};
        if (not normalized_position.has_value()) {
            continue;  // LCOV_EXCL_LINE
        }
        auto const [u_i, i]{normalized_position.value()};

        std::vector<double const*> parameter_blocks;
        parameter_blocks.push_back(camera_state.intrinsics.data());
        for (int j{0}; j < 4; ++j) {
            parameter_blocks.push_back(spline.ControlPoints().col(i + j).data());
        }

        auto const& [pixels, points]{targets.at(timestamp_ns).bundle};
        Eigen::Array<double, Eigen::Dynamic, 2, Eigen::RowMajor> residuals_i{pixels.rows(), 2};
        for (Eigen::Index j{0}; j < pixels.rows(); ++j) {
            ceres::CostFunction const* const cost_function{cost_functions::Create(sensor.camera_model, sensor.bounds,
                                                                                  pixels.row(j), points.row(j), u_i,
                                                                                  spline.GetTimeHandler().delta_t_ns_)};

            cost_function->Evaluate(parameter_blocks.data(), residuals_i.row(j).data(), nullptr);
        }

        residuals.insert({timestamp_ns, residuals_i});
    }

    return {poses, residuals};
}  // LCOV_EXCL_LINE

ImuErrors EvaluateImuError(ImuMeasurements const& imu_data, ImuCamExtrinsic const& extrinsic,
                           spline::Se3Spline const& spline) {
    ImuErrors imu_residuals;

    for (auto const timestamp_ns : imu_data | std::views::keys) {
        // TODO(Jack): This logic is now repeated several times... we are missing the point I think. How to fix!?
        auto const normalized_position{spline.GetTimeHandler().SplinePosition(timestamp_ns, spline.Size())};
        if (not normalized_position.has_value()) {
            continue;  // LCOV_EXCL_LINE
        }
        auto const [u_i, i]{normalized_position.value()};

        std::vector<double const*> parameter_blocks;
        parameter_blocks.push_back(extrinsic.tf.se3_a_b.data());
        for (int j{0}; j < 4; ++j) {
            parameter_blocks.push_back(spline.ControlPoints().col(i + j).data());
        }
        ceres::CostFunction const* const cost_function_1{cost_functions::RigidBodyAngularVelocity::Create(
            imu_data.at(timestamp_ns).angular_velocity, u_i, spline.GetTimeHandler().delta_t_ns_)};

        Vector6d residual_i;
        cost_function_1->Evaluate(parameter_blocks.data(), residual_i.topRows<3>().data(), nullptr);

        parameter_blocks.insert(std::begin(parameter_blocks) + 1, extrinsic.gravity.data());
        ceres::CostFunction const* const cost_function_2{cost_functions::RigidBodyLinearAcceleration::Create(
            imu_data.at(timestamp_ns).linear_acceleration, u_i, spline.GetTimeHandler().delta_t_ns_)};

        cost_function_2->Evaluate(parameter_blocks.data(), residual_i.bottomRows<3>().data(), nullptr);

        imu_residuals.insert({timestamp_ns, {residual_i.topRows<3>(), residual_i.bottomRows<3>()}});
    }

    return imu_residuals;
}  // LCOV_EXCL_LINE

}  // namespace  reprojection::optimization
