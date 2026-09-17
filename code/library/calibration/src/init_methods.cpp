#include "calibration/init_methods.hpp"

#include <algorithm>
#include <map>
#include <ranges>
#include <vector>

#include "geometry/lie.hpp"
#include "logging/logging.hpp"
#include "optimization/angular_velocity_alignment.hpp"
#include "optimization/bundle_adjustment.hpp"
#include "projection_functions/initialize_camera.hpp"
#include "time_sync/time_sync.hpp"

#include "camera_pose_init.hpp"
#include "intrinsic_initialization.hpp"
#include "utilities.hpp"
#include "visual_inertial_init.hpp"

namespace reprojection::calibration {

namespace {

auto const log{logging::Get("calibration")};

}

std::optional<ArrayXd> InitializeIntrinsics(CameraModel const camera_model, std::optional<double> const& focal_length,
                                            double const height, double const width, TargetSamples const& targets,
                                            int const num_threads) {
    // Generate all focal length initialization values. There is an automatic policy of (0 -> max(h,w)) or a
    // configurable focal_length plus minus 0.5xfocal_length.
    double const max_dim{std::max(height, width)};
    double const min_focal_length{focal_length ? *focal_length * 0.5 : 0.0};
    double const max_focal_length{focal_length ? *focal_length * 1.5 : max_dim};

    constexpr int num_samples{200};
    std::vector<double> focal_lengths;
    for (int i{0}; i < num_samples; ++i) {
        double const t{static_cast<double>(i) / (num_samples - 1)};
        focal_lengths.push_back(min_focal_length + (t * (max_focal_length - min_focal_length)));
    }

    // Generate a subset of targets which we will use to test our intrinsic hypothesis with.
    auto const target_subset{SampleMap(targets, 20)};

    // Evaluate small bundle adjustment problems for each provided focal length and pick the one that has the lowest
    // cost.
    auto const initializer{SelectInitializationStrategy(camera_model)};
    std::map<double, Intrinsic> cost_intrinsic_map;
    for (auto const f_i : focal_lengths) {
        CameraInfo const camera_info{camera_model, {0, width, 0, height}};
        Intrinsic const intrinsics_i{initializer(f_i, height, width)};

        Frames const initial_poses{PoseInitialization(camera_info, target_subset, intrinsics_i)};
        // TODO(Jack): Is the required success rate used in this condition enough, too much, or too little?
        if (std::size(initial_poses) < 0.8 * std::size(target_subset)) {
            continue;  // LCOV_EXCL_LINE
        }

        // Do a bundle adjustment with the intrinsics constant and calculate the mean residual. Our hope is that the
        // intrinsic which will be the best initialization for the full optimization will produce the lowest mean
        // residual here on a subset of targets.
        auto const problem{optimization::BundleAdjustment::SingleCamProblem(camera_info, intrinsics_i, target_subset,
                                                                            initial_poses, false, AssetId{0})};
        auto const [_, ceres_state]{optimization::BundleAdjustment::Solve(problem, num_threads)};

        double const mean_residual{ceres_state.solver_summary.final_cost / ceres_state.solver_summary.num_residuals};
        cost_intrinsic_map[mean_residual] = intrinsics_i;

        log->debug("{{'focal_length': {}, 'mean_residual': {}, 'num_frames_used': {}}}", f_i, mean_residual,
                   std::size(initial_poses));
    }

    if (std::size(cost_intrinsic_map) == 0) {
        return std::nullopt;  // LCOV_EXCL_LINE
    } else {
        // Take the intrinsic with the lowest mean residual.
        return std::cbegin(cost_intrinsic_map)->second.value;
    }
}

// Doxygen notes: only work because we have same camera center for the pinhole and ds/other camera model used. The goal
// of the function is to unproject the pixels to 3d rays using a roughly initialized camera, then project these back to
// pixels using an ideal unit pinhole camera, which essentially undistorts them. Now that we have data that comes from
// an equivalent pinhole camera we can apply dlt/pnp and get an initial pose.
Frames PoseInitialization(CameraInfo const& camera_info, TargetSamples const& targets, Intrinsic const& intrinsic) {
    auto const camera{
        projection_functions::InitializeCamera(camera_info.camera_model, intrinsic.value, camera_info.bounds)};

    Frames frames;
    for (auto const& [timestamp_ns, target_i] : targets) {
        auto const pose{EstimatePoseViaPinholePnP(camera, target_i.bundle, camera_info.bounds)};
        if (pose.has_value()) {
            frames[timestamp_ns] = *pose;
        }
    }

    return frames;
}  // LCOV_EXCL_LINE

// NOTE(Jack): This method depends on the fact that the both frame sets are calculated with respect to the same target
// or world coordinate frame. For single target calibration data acquisitions this is the case, but if we ever move more
// complicated scenarios we need to remember that assumption is built in here.
// TODO(Jack): Do we need to do some sort of validation for the input frames? I.e. that they are not both empty or do
// not have any synchronized matches? At this point I am not sure how we would express such a failure in the step, but
// the lack of frames or lack of sync-ability is a very real risk.
Array6d StereoExtrinsicInit(Frames const& frames_a, Frames const& frames_b, uint64_t const approx_sync_delta_ns) {
    auto const timestamps{frames_b | std::views::keys};
    std::set<uint64_t> remaining_b{std::cbegin(timestamps), std::cend(timestamps)};

    Array6d se3_coa_cob{Array6d::Zero()};
    int num_synced{0};
    for (auto const& [a_timestamp_ns, pose_a] : frames_a) {
        // TODO(Jack): The fact that we hand roll the time sync logic here is not so nice. There are a couple places we
        // need this same logic and we need to consider how to unify them into a central implementation if we start
        // duplicating code.
        // TODO(Jack): We need to find a way to unit test this time sync code!
        auto const b_timestamp_it{time_sync::FindClosest(remaining_b, a_timestamp_ns)};
        if (b_timestamp_it == std::cend(remaining_b)) {
            continue;  // LCOV_EXCL_LINE
        }

        auto const b_timestamp_ns{*b_timestamp_it};
        if (not time_sync::IsWithinThreshold(b_timestamp_ns, a_timestamp_ns, approx_sync_delta_ns)) {
            continue;  // LCOV_EXCL_LINE
        }

        // Remove it so a double match cannot happen.
        remaining_b.erase(b_timestamp_it);

        // This is now the pose_b pose that is synchronized to the current pose_a given the max_sync_delta_ns
        // tolerance.
        Pose const pose_b{frames_b.at(b_timestamp_ns)};

        // "coa" = "camera optical a" and "cob" = "camera optical b"
        Isometry3d const tf_coa_w{geometry::Exp(pose_a.value)};
        Isometry3d const tf_cob_w{geometry::Exp(pose_b.value)};
        Isometry3d const tf_coa_cob{tf_coa_w * tf_cob_w.inverse()};
        Array6d const se3_coa_cob_i{geometry::Log(tf_coa_cob)};

        se3_coa_cob += se3_coa_cob_i;
        num_synced++;
    }

    std::string const log_msg{
        std::format("{{ 'num_frames_a': {}, 'num_frames_b': {}, 'num_frames_synced': {}, 'approx_sync_delta_ns': {}}}",
                    std::size(frames_a), std::size(frames_b), num_synced, approx_sync_delta_ns)};
    if (num_synced == 0) {
        log->error(log_msg);  // LCOV_EXCL_LINE

        // TODO(Jack): Is this legitimate to return an identity transform if the initialization otherwise failed due to
        // failed syncing?
        return Array6d::Zero();
    } else {
        log->debug(log_msg);

        // TODO(Jack): Is this legitimate to average the transforms like this?
        return se3_coa_cob / num_synced;
    }
}

std::pair<std::pair<Array3d, CeresState>, Vector3d> EstimateCameraImuAlignment(spline::Se3Spline const& spline,
                                                                               ImuSamples const& imu_data,
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
