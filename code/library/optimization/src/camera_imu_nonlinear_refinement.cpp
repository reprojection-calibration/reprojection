#include "optimization/camera_imu_nonlinear_refinement.hpp"

#include <ranges>

#include "spline/spline_initialization.hpp"

#include "spline_projection_cost_function.hpp"

namespace reprojection::optimization {

ReprojectionErrors SplineReprojectionResiduals(CameraInfo const& sensor, CameraMeasurements const& targets,
                                               CameraState const& camera_state, spline::Se3Spline const& spline) {
    // TODO(Jack): We are calculating the reprojection errors for all targets that are on the interpolated spline. That
    //  means that even if there is no initial pose that we will have an evaluation. This means there can be no foreign
    //  key constraint. Do we need new tables for this?
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

    return residuals;
}  // LCOV_EXCL_LINE

}  // namespace  reprojection::optimization
