#include "optimization/spline_reprojection_error.hpp"

#include <ranges>

#include "spline/spline_initialization.hpp"

#include "spline_projection_cost_function.hpp"

namespace reprojection::optimization {

ReprojectionErrors SplineReprojectionResiduals(CameraInfo const& sensor, CameraMeasurements const& targets,
                                               CameraState const& camera_state,
                                               std::pair<spline::Matrix2NXd, spline::TimeHandler> const& spline) {
    auto const& [control_points, time_handler]{spline};

    // TODO(Jack): We are calculating the reprojection errors for all targets that are on the interpolated spline. That
    //  means that even if there is no initial pose that we will have an evaluation. This means there can be no foreign
    //  key constraint. Do we need new tables for this?
    ReprojectionErrors residuals;
    for (auto const timestamp_ns : targets | std::views::keys) {
        auto const normalized_position{time_handler.SplinePosition(timestamp_ns, control_points.cols())};
        if (not normalized_position.has_value()) {
            std::cout << "Skipping: " << timestamp_ns << std::endl;
            continue;
        }
        auto const [u_i, i]{normalized_position.value()};

        std::vector<double const*> parameter_blocks;
        parameter_blocks.push_back(camera_state.intrinsics.data());
        parameter_blocks.push_back(control_points.col(i).data());
        parameter_blocks.push_back(control_points.col(i + 1).data());
        parameter_blocks.push_back(control_points.col(i + 2).data());
        parameter_blocks.push_back(control_points.col(i + 3).data());

        auto const& [pixels, points]{targets.at(timestamp_ns).bundle};
        Eigen::Array<double, Eigen::Dynamic, 2, Eigen::RowMajor> residuals_i{pixels.rows(), 2};
        for (Eigen::Index j{0}; j < pixels.rows(); ++j) {
            ceres::CostFunction const* const cost_function{Create(sensor.camera_model, sensor.bounds, pixels.row(j),
                                                                  points.row(j), u_i, time_handler.delta_t_ns_)};

            cost_function->Evaluate(parameter_blocks.data(), residuals_i.row(j).data(), nullptr);
        }

        residuals.insert({timestamp_ns, residuals_i});
    }

    return residuals;
}

}  // namespace  reprojection::optimization
