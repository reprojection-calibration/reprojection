#include "optimization/camera_imu_nonlinear_refinement.hpp"

#include <ranges>

#include "spline/spline_initialization.hpp"
#include "spline/spline_state.hpp"
#include "types/spline_types.hpp"

#include "spline_projection_cost_function.hpp"

namespace reprojection::optimization {

std::tuple<OptimizationState, CeresState> CameraImuNonlinearRefinement(CameraInfo const& sensor,
                                                                       CameraMeasurements const& targets,
                                                                       ImuMeasurements const& imu_data,
                                                                       OptimizationState const& initial_state) {
    (void)sensor;
    (void)imu_data;

    // Spline pose init noise
    PositionMeasurements camera_orientations;
    PositionMeasurements camera_positions;
    for (auto const& [timestamp_ns, frame_i] : initial_state.frames) {
        camera_orientations.insert({timestamp_ns, {frame_i.pose.topRows(3)}});
        camera_positions.insert({timestamp_ns, {frame_i.pose.bottomRows(3)}});
    }
    int const interpolation_density{2};
    spline::CubicBSplineC3 const camera_orientation_spline{
        spline::InitializeC3Spline(camera_orientations, interpolation_density * std::size(initial_state.frames))};
    spline::CubicBSplineC3 const camera_position_spline{
        spline::InitializeC3Spline(camera_positions, interpolation_density * std::size(initial_state.frames))};

    std::vector<Array6d> interpolated_camera_spline;
    interpolated_camera_spline.reserve(std::size(camera_orientation_spline.control_points));
    for (size_t i{0}; i < std::size(camera_orientation_spline.control_points); ++i) {
        Array6d pose_i;
        pose_i << camera_orientation_spline.control_points[i], camera_position_spline.control_points[i];
    }

    // Classic optimization stuff
    CeresState ceres_state{ceres::TAKE_OWNERSHIP, ceres::DENSE_SCHUR};
    ceres::Problem problem{ceres_state.problem_options};

    CameraState optimized_camera_state{initial_state.camera_state};
    for (auto const timestamp_ns : targets | std::views::keys) {
        auto const normalized_position{camera_orientation_spline.time_handler.SplinePosition(
            timestamp_ns, std::size(camera_orientation_spline.control_points))};
        if (not normalized_position.has_value()) {
            std::cout << "Skipped timestamp: " << timestamp_ns << std::endl;
            continue;
        }
        auto const [u_i, i]{normalized_position.value()};

        auto const& [pixels, points]{targets.at(timestamp_ns).bundle};

        for (Eigen::Index j{0}; j < pixels.rows(); ++j) {
            ceres::CostFunction* const cost_function{Create(sensor.camera_model, sensor.bounds, pixels.row(j),
                                                            points.row(j), u_i,
                                                            camera_orientation_spline.time_handler.delta_t_ns_)};

            problem.AddResidualBlock(cost_function, nullptr, optimized_camera_state.intrinsics.data(),
                                     interpolated_camera_spline[i].data(), interpolated_camera_spline[i + 1].data(),
                                     interpolated_camera_spline[i + 2].data(),
                                     interpolated_camera_spline[i + 3].data());
        }
    }

    ceres_state.solver_options.minimizer_progress_to_stdout = true;
    ceres::Solve(ceres_state.solver_options, &problem, &ceres_state.solver_summary);

    std::cout << ceres_state.solver_summary.FullReport() << std::endl;

    return {{}, {}};
}

}  // namespace  reprojection::optimization
