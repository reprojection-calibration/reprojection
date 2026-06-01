#include "calibration/initialization_methods.hpp"

#include <algorithm>
#include <map>
#include <ranges>
#include <vector>

#include "geometry/lie.hpp"
#include "logging/logging.hpp"
#include "optimization/angular_velocity_alignment.hpp"
#include "optimization/bundle_adjustment.hpp"
#include "projection_functions/initialize_camera.hpp"

#include "camera_imu_initialization.hpp"
#include "intrinsic_initialization.hpp"
#include "linear_pose_initialization.hpp"
#include "utilities.hpp"

namespace reprojection::calibration {

namespace {

auto const log{logging::Get("calibration")};

}

// TODO(Jack): Should we parameterize the minimum number of samples (num_samples) and should we parameterize the number
// of targets sampled?
//
std::optional<ArrayXd> InitializeIntrinsics(CameraModel const camera_model, double const height, double const width,
                                            CameraMeasurements const& targets) {
    auto const [runner, initialization]{SelectInitializationStrategy(camera_model, height, width)};

    // Generate gamma estimates
    std::vector<double> gammas;
    for (auto const& target : targets | std::views::values) {
        std::vector<double> const gammas_i{runner(target)};
        gammas.insert(std::cend(gammas), std::cbegin(gammas_i), std::cend(gammas_i));
    }
    std::sort(std::begin(gammas), std::end(gammas));

    // Test a uniform sampling of the gamma estimates using small randomly sampled pose-only optimizations.
    uint64_t const num_samples{std::min<uint64_t>(std::size(gammas), 500)};
    std::map<double, ArrayXd> cost_intrinsic_map;
    for (uint64_t i{0}; i < num_samples; ++i) {
        uint64_t const idx{i * std::size(gammas) / num_samples};
        double const gamma_i{gammas[idx]};

        // Sample target subset and initialize poses
        CameraInfo const camera_info{"", camera_model, {0, width, 0, height}};
        auto const target_subset{SampleMap(targets, 10)};
        ArrayXd const intrinsics_i{initialization(gamma_i, height, width)};
        Frames const initial_poses{LinearPoseInitialization(camera_info, target_subset, {intrinsics_i})};

        if (std::size(initial_poses) == 0) {
            continue;  // LCOV_EXCL_LINE
        }

        // Do nonlinear refinement with the intrinsics constant
        OptimizationState const initial_state{{intrinsics_i}, initial_poses};
        auto const [optimized_state,
                    diagnostics]{optimization::BundleAdjustment(camera_info, target_subset, initial_state, true)};
        cost_intrinsic_map[diagnostics.solver_summary.final_cost] = intrinsics_i;

        log->debug("{{ 'idx': {}, 'gamma': {}, 'final_cost': {}, 'num_frames_used': {}}}", idx, gamma_i,
                   diagnostics.solver_summary.final_cost, std::size(initial_poses));
    }

    if (std::size(cost_intrinsic_map) == 0) {
        return std::nullopt;  // LCOV_EXCL_LINE
    } else {
        // Take the intrinsic with the lowest final cost.
        return std::cbegin(cost_intrinsic_map)->second;
    }
}

// Doxygen notes: only work because we have same camera center for the pinhole and ds/other camera model used. The goal
// of the function is to unproject the pixels to 3d rays using a roughly initialized camera, then project these back to
// pixels using an ideal unit pinhole camera, which essentially undistorts them. Now that we have data that comes from
// an equivalent pinhole camera we can apply dlt/pnp and get an initial pose.
// TODO(Jack): This name is misleading because the process is not actually strictly linear!
Frames LinearPoseInitialization(CameraInfo const& sensor, CameraMeasurements const& targets,
                                CameraState const& intrinsics) {
    auto const camera{
        projection_functions::InitializeCamera(sensor.camera_model, intrinsics.intrinsics, sensor.bounds)};

    Frames linear_solution;
    for (auto const& [timestamp_ns, target_i] : targets) {
        auto const pose{EstimatePoseViaPinholePnP(camera, target_i.bundle, sensor.bounds)};
        if (pose.has_value()) {
            linear_solution[timestamp_ns] = *pose;
        }
    }

    return linear_solution;
}  // LCOV_EXCL_LINE

std::tuple<std::tuple<Matrix3d, CeresState>, Vector3d> EstimateCameraImuRotationAndGravity(
    spline::CubicBSplineC3 const& camera_orientation, ImuMeasurements const& imu_data) {
    auto const imu_angular_velocity{ExtractAngularVelocity(imu_data)};
    auto const [aa_imu_co,
                diagnostics]{optimization::AngularVelocityAlignment(imu_angular_velocity, camera_orientation)};

    Matrix3d const R_imu_co{geometry::Exp<double>(aa_imu_co)};

    auto const imu_linear_acceleration{ExtractLinearAcceleration(imu_data)};
    Vector3d const gravity_w{EstimateGravity(camera_orientation, imu_linear_acceleration, R_imu_co)};

    return {{R_imu_co, diagnostics}, gravity_w};
}

}  // namespace reprojection::calibration
