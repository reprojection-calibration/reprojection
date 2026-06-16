#include "optimization/camera_imu_calibration.hpp"

#include <ranges>

#include "cost_functions/reprojection_error_spline.hpp"
#include "cost_functions/rigid_body_angular_velocity.hpp"
#include "cost_functions/rigid_body_linear_acceleration.hpp"
#include "spline/spline_initialization.hpp"

namespace reprojection::optimization {

std::pair<Frames, ReprojectionErrors> ReprojectionErrorSpline(CameraInfo const& sensor,
                                                              CameraMeasurements const& targets,
                                                              CameraState const& camera_state,
                                                              spline::Se3Spline const& spline_w_co) {
    // TODO(Jack): We are calculating the reprojection errors for all targets that are on the interpolated spline. That
    //  means that even if there is no initial pose that we will have an evaluation. This means there can be no foreign
    //  key constraint. Do we need new tables for this?
    Frames tf_co_w;
    ReprojectionErrors residuals;
    for (auto const timestamp_ns : targets | std::views::keys) {
        auto const tf_w_co_i{spline_w_co.Evaluate(timestamp_ns, spline::DerivativeOrder::Null)};
        if (not tf_w_co_i) {
            continue;
        }

        // NOTE(Jack): Our convention is to store in the database camera poses that takes points in the world frame and
        // convert them into the camera optical frame. But the spline produces the opposite transform so we need to
        // invert it here.
        Array6d const tf_co_w_i{geometry::InverseTransform<double>(*tf_w_co_i)};
        tf_co_w.insert({timestamp_ns, {tf_co_w_i}});

        auto const normalized_position{spline_w_co.GetTimeHandler().SplinePosition(timestamp_ns, spline_w_co.Size())};
        if (not normalized_position.has_value()) {
            continue;  // LCOV_EXCL_LINE
        }
        auto const [u_i, i]{normalized_position.value()};

        std::vector<double const*> parameter_blocks;
        parameter_blocks.push_back(camera_state.intrinsics.data());
        for (int j{0}; j < 4; ++j) {
            parameter_blocks.push_back(spline_w_co.ControlPoints().col(i + j).data());
        }

        auto const& [pixels, points]{targets.at(timestamp_ns).bundle};
        Eigen::Array<double, Eigen::Dynamic, 2, Eigen::RowMajor> residuals_i{pixels.rows(), 2};
        for (Eigen::Index j{0}; j < pixels.rows(); ++j) {
            ceres::CostFunction const* const cost_function{
                cost_functions::Create(sensor.camera_model, sensor.bounds, pixels.row(j), points.row(j), u_i,
                                       spline_w_co.GetTimeHandler().delta_t_ns_)};

            cost_function->Evaluate(parameter_blocks.data(), residuals_i.row(j).data(), nullptr);
        }

        residuals.insert({timestamp_ns, residuals_i});
    }

    return {tf_co_w, residuals};
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
