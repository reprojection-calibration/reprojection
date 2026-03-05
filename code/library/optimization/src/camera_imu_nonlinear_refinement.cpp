#include "optimization/camera_imu_nonlinear_refinement.hpp"

#include <ranges>

#include "spline/spline_initialization.hpp"

#include "spline_projection_cost_function.hpp"

namespace reprojection::optimization {

std::tuple<std::pair<CameraState, spline::Se3Spline>, CeresState> SplineNonlinearRefinement(
    CameraInfo const& sensor, CameraMeasurements const& targets, CameraState const& camera_state,
    spline::Se3Spline const& spline) {
    CeresState ceres_state{ceres::TAKE_OWNERSHIP, ceres::SPARSE_SCHUR};
    ceres::Problem problem{ceres_state.problem_options};

    CameraState optimized_camera_state{camera_state};
    spline::Matrix2NXd optimized_control_points{spline.ControlPoints()};
    for (auto const timestamp_ns : targets | std::views::keys) {
        auto const normalized_position{
            spline.GetTimeHandler().SplinePosition(timestamp_ns, optimized_control_points.cols())};
        if (not normalized_position.has_value()) {
            continue;  // LCOV_EXCL_LINE
        }
        auto const [u_i, i]{normalized_position.value()};

        auto const& [pixels, points]{targets.at(timestamp_ns).bundle};
        for (Eigen::Index j{0}; j < pixels.rows(); ++j) {
            ceres::CostFunction* const cost_function{Create(sensor.camera_model, sensor.bounds, pixels.row(j),
                                                            points.row(j), u_i, spline.GetTimeHandler().delta_t_ns_)};

            problem.AddResidualBlock(cost_function, nullptr, optimized_camera_state.intrinsics.data(),
                                     optimized_control_points.col(i).data(), optimized_control_points.col(i + 1).data(),
                                     optimized_control_points.col(i + 2).data(),
                                     optimized_control_points.col(i + 3).data());
        }
    }

    ceres_state.solver_options.minimizer_progress_to_stdout = true;
    ceres_state.solver_options.num_threads = 6; // DO NOT HARDCODE!!!
    ceres::Solve(ceres_state.solver_options, &problem, &ceres_state.solver_summary);
    std::cout << ceres_state.solver_summary.FullReport() << "\n";

    spline::Se3Spline const optimized_spline{optimized_control_points, spline.GetTimeHandler()};

    return {{optimized_camera_state, optimized_spline}, ceres_state};
}

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
