#include "optimization/camera_imu_nonlinear_refinement.hpp"

#include <ranges>

#include "spline/spline_initialization.hpp"

#include "spline_projection_cost_function.hpp"

namespace reprojection::optimization {



// TODO(Jack): Naming to reflect the fact that it also returns the poses!
std::pair<Frames, ReprojectionErrors> SplineReprojectionResiduals(CameraInfo const& sensor,
                                                                  CameraMeasurements const& targets,
                                                                  CameraState const& camera_state,
                                                                  spline::Se3Spline const& spline) {
    Frames poses;
    for (auto const timestamp_ns : targets | std::views::keys) {
        auto const pose_i{spline.Evaluate(timestamp_ns, spline::DerivativeOrder::Null)};
        if (pose_i) {
            poses[timestamp_ns].pose = pose_i.value();
        }
    }

    ReprojectionErrors residuals;
    for (auto const timestamp_ns : targets | std::views::keys) {
        auto const normalized_position{
            spline.GetTimeHandler().SplinePosition(timestamp_ns, spline.ControlPoints().cols())};
        if (not normalized_position.has_value()) {
            continue;  // LCOV_EXCL_LINE
        }
        auto const [u_i, i]{normalized_position.value()};

        std::vector<double const*> parameter_blocks;
        parameter_blocks.push_back(camera_state.intrinsics.data());
        parameter_blocks.push_back(spline.ControlPoints().col(i).data());
        parameter_blocks.push_back(spline.ControlPoints().col(i + 1).data());
        parameter_blocks.push_back(spline.ControlPoints().col(i + 2).data());
        parameter_blocks.push_back(spline.ControlPoints().col(i + 3).data());

        auto const& [pixels, points]{targets.at(timestamp_ns).bundle};
        Eigen::Array<double, Eigen::Dynamic, 2, Eigen::RowMajor> residuals_i{pixels.rows(), 2};
        for (Eigen::Index j{0}; j < pixels.rows(); ++j) {
            ceres::CostFunction const* const cost_function{Create(sensor.camera_model, sensor.bounds, pixels.row(j),
                                                                  points.row(j), u_i,
                                                                  spline.GetTimeHandler().delta_t_ns_)};

            cost_function->Evaluate(parameter_blocks.data(), residuals_i.row(j).data(), nullptr);
        }

        residuals.insert({timestamp_ns, residuals_i});
    }

    return {poses, residuals};
}  // LCOV_EXCL_LINE

}  // namespace  reprojection::optimization
