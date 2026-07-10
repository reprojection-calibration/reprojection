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

#include "extrinsic_initialization.hpp"
#include "intrinsic_initialization.hpp"
#include "pose_initialization.hpp"
#include "utilities.hpp"

namespace reprojection::calibration {

namespace {

auto const log{logging::Get("calibration")};

}

// TODO(Jack): Should we parameterize the minimum number of samples (num_samples) and should we parameterize the number
// of targets sampled?
//
std::optional<ArrayXd> InitializeIntrinsics(CameraModel const camera_model, double const height, double const width,
                                            CameraMeasurements const& targets, int const num_threads) {
    auto const [runner, initialization]{SelectInitializationStrategy(camera_model, height, width)};

    // Generate all gamma estimates and sort them in ascending order.
    std::vector<double> gammas;
    for (auto const& target : targets | std::views::values) {
        std::vector<double> const gammas_i{runner(target)};
        gammas.insert(std::cend(gammas), std::cbegin(gammas_i), std::cend(gammas_i));
    }
    std::sort(std::begin(gammas), std::end(gammas));

    // Generate a subset of targets which we will use to test our intrinsic hypothesis with.
    //
    // TODO(Jack): Is 20 enough, too many, or too little?
    // TODO(Jack): What if the set of selected targets has bad properties like too many outliers or other degnerate
    // cases for a camera calibration bundle adjustment. How would the user be able to get around this point? We should
    // offer the user the option to manually initialize the intrinsics.
    auto const target_subset{SampleMap(targets, 20)};

    // Sample the gammas evenly (this narrows down how many evaluations we need to do) and calculate the residual from a
    // pose only bundle adjustment using intrinsics initialized from the gamma value. The gamme which produces the
    // lowest residual will be our choice as the best initialization value.
    //
    // TODO(Jack): What is the maximum number of samples we need to take here. At time of writing (09.07.2026) 500 seems
    // like a lot and could slow the process down on a slow computer. We need to do some testing I think.
    uint64_t const num_samples{std::min<uint64_t>(std::size(gammas), 500)};
    std::map<double, ArrayXd> cost_intrinsic_map;
    for (uint64_t i{0}; i < num_samples; ++i) {
        uint64_t const idx{i * std::size(gammas) / num_samples};

        double const gamma_i{gammas[idx]};
        CameraInfo const camera_info{"", camera_model, {0, width, 0, height}};
        ArrayXd const intrinsics_i{initialization(gamma_i, height, width)};

        Frames const initial_poses{PoseInitialization(camera_info, target_subset, {intrinsics_i})};
        // TODO(Jack): Is the required success rate used in this condition enough, too much, or too little?
        if (std::size(initial_poses) < 0.8 * std::size(target_subset)) {
            continue;  // LCOV_EXCL_LINE
        }

        // Do a bundle adjustment with the intrinsics constant and calculate the mean residual. Our hope is that the
        // intrinsic which will be the best initialization for the full optimization will produce the lowest mean
        // residual here on a subset of targets.
        OptimizationState const initial_state{{intrinsics_i}, initial_poses};
        auto const [optimized_state, diagnostics]{
            optimization::BundleAdjustment(camera_info, target_subset, initial_state, num_threads, true)};

        double const mean_residual{diagnostics.solver_summary.final_cost / diagnostics.solver_summary.num_residuals};
        cost_intrinsic_map[mean_residual] = intrinsics_i;

        log->debug("{{ 'idx': {}, 'gamma': {}, 'mean_residual': {}, 'num_frames_used': {}}}", idx, gamma_i,
                   mean_residual, std::size(initial_poses));
    }

    if (std::size(cost_intrinsic_map) == 0) {
        return std::nullopt;  // LCOV_EXCL_LINE
    } else {
        // Take the intrinsic with the lowest mean residual.
        return std::cbegin(cost_intrinsic_map)->second;
    }
}

// Doxygen notes: only work because we have same camera center for the pinhole and ds/other camera model used. The goal
// of the function is to unproject the pixels to 3d rays using a roughly initialized camera, then project these back to
// pixels using an ideal unit pinhole camera, which essentially undistorts them. Now that we have data that comes from
// an equivalent pinhole camera we can apply dlt/pnp and get an initial pose.
Frames PoseInitialization(CameraInfo const& camera_info, CameraMeasurements const& targets,
                          CameraState const& intrinsics) {
    auto const camera{
        projection_functions::InitializeCamera(camera_info.camera_model, intrinsics.intrinsics, camera_info.bounds)};

    Frames frames;
    for (auto const& [timestamp_ns, target_i] : targets) {
        auto const pose{EstimatePoseViaPinholePnP(camera, target_i.bundle, camera_info.bounds)};
        if (pose.has_value()) {
            frames[timestamp_ns] = *pose;
        }
    }

    return frames;
}  // LCOV_EXCL_LINE

std::pair<std::pair<Array3d, CeresState>, Vector3d> EstimateCameraImuAlignment(spline::Se3Spline const& spline,
                                                                               ImuMeasurements const& imu_data,
                                                                               int const num_threads) {
    auto const imu_angular_velocity{ExtractAngularVelocity(imu_data)};
    auto const [aa_imu_co,
                diagnostics]{optimization::AngularVelocityAlignment(imu_angular_velocity, spline, num_threads)};

    Matrix3d const R_imu_co{geometry::Exp<double>(aa_imu_co)};
    auto const imu_linear_acceleration{ExtractLinearAcceleration(imu_data)};
    Vector3d const gravity_w{
        EstimateGravity({spline.So3(), spline.GetTimeHandler()}, imu_linear_acceleration, R_imu_co)};

    return {{aa_imu_co, diagnostics}, gravity_w};
}

}  // namespace reprojection::calibration
