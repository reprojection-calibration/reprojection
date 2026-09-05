#include "optimization/bundle_adjustment.hpp"

#include <ceres/loss_function.h>

#include <ranges>

#include "cost_functions/reprojection_error.hpp"

namespace reprojection::optimization {

// ERROR(Jack): What is a frame has too few valid pixels to actually constrain the pose? Should we entirely skip
// that frame? Or what if in general we have a minimum required of points per frame threshold?
BundleAdjustment::Result BundleAdjustment::Solve(Problem const& ba_problem, int const num_threads) {
    // TODO(Jack): It is a little messy how we construct the result from just part of the problem, and then iterate over
    // the problem below but ignore the part that we copied to the result and use the result instead. Really not the end
    // of the world but I feel like I am missing the plotline.
    Result result{ba_problem};
    result.ceres_state.solver_options.num_threads = num_threads;
    ceres::Problem ceres_problem{result.ceres_state.problem_options};

    for (auto const& [camera_id, _, frame_timestamp_ns, bundle] : ba_problem.observations) {
        // cppcheck-suppress ignoredReturnValue
        auto const& [camera_info, _1, camera_options]{ba_problem.cameras.at(camera_id)};
        auto& camera_state{result.camera_states.at(camera_id)};
        // Protect against the case of a missing rig pose - it can be that we have a observation for a frame where the
        // rig pose initialization was unsuccessful and we need to protect against that.
        if (not result.rig_poses.contains(frame_timestamp_ns)) {
            continue;  // LCOV_EXCL_LINE
        }
        auto& rig_pose{result.rig_poses.at(frame_timestamp_ns)};

        auto const& [pixels, points]{bundle};
        for (Eigen::Index j{0}; j < pixels.rows(); ++j) {
            ceres::CostFunction* const cost_function{
                cost_functions::Create(camera_info.camera_model, camera_info.bounds, pixels.row(j), points.row(j))};

            ceres_problem.AddResidualBlock(cost_function, new ceres::HuberLoss(1.0),
                                           camera_state.intrinsic.value.data(), camera_state.extrinsic.data(),
                                           rig_pose.value.data());
        }

        if (not camera_options.optimize_intrinsic) {
            ceres_problem.SetParameterBlockConstant(camera_state.intrinsic.value.data());
        }
        if (not camera_options.optimize_extrinsic) {
            ceres_problem.SetParameterBlockConstant(camera_state.extrinsic.data());
        }
    }

    ceres::Solve(result.ceres_state.solver_options, &ceres_problem, &result.ceres_state.solver_summary);

    return result;
}

BundleAdjustment::Problem BundleAdjustment::MultiCamProblem(std::vector<CameraProblemInput> const& cameras,
                                                            Frames const& rig_poses, uint64_t max_sync_delta_ns) {
    auto const& reference_cam{cameras.front()};
    Problem problem{SingleCamProblem(reference_cam.camera_info, reference_cam.intrinsic, reference_cam.targets,
                                     rig_poses, reference_cam.optimize_intrinsic, reference_cam.camera_id)};

    for (auto const& camera : cameras | std::views::drop(1)) {
        AddCamera(camera, max_sync_delta_ns, camera.optimize_intrinsic, camera.optimize_extrinsic, problem);
    }

    return problem;
}

BundleAdjustment::Problem BundleAdjustment::SingleCamProblem(CameraInfo const& camera_info, Intrinsic const& intrinsic,
                                                             TargetSamples const& targets, Frames const& frames,
                                                             bool const optimize_intrinsic, AssetId const camera_id) {
    // NOTE(Jack): For a single camera problems we do not consider the rig-cam extrinsic. Therefore we set it to
    // identity (i.e. Array6d::Zero()) and set optimize_extrinsic to false. This is essentially a central characteristic
    // of the single cam problem.
    Camera const camera{camera_info, CameraState{intrinsic, Array6d::Zero()}, CameraOptions{optimize_intrinsic, false}};

    std::vector<Observation> observations;
    for (auto const& [timestamp_ns, target] : targets) {
        // NOTE(Jack): For the single cam problems the data is by its very nature "synchronized", therefore we use the
        // same timestamp for both observation timestamps.
        observations.push_back({camera_id, timestamp_ns, timestamp_ns, target.bundle});
    }

    return {{{camera_id, camera}}, frames, observations};
}

BundleAdjustment::Problem BundleAdjustment::SingleFrameProblem(CameraInfo const& camera_info,
                                                               Intrinsic const& intrinsic, Bundle const& bundle,
                                                               Pose const& pose, bool const optimize_intrinsic) {
    uint64_t constexpr timestamp_ns{0};
    ExtractedTarget const target{bundle, {}};
    AssetId const camera_id{0};

    return SingleCamProblem(camera_info, intrinsic, TargetSamples{{timestamp_ns, target}}, Frames{{timestamp_ns, pose}},
                            optimize_intrinsic, camera_id);
}

// TODO TEST? and split between hpp and cpp?
auto FindClosest(TargetSamples const& data, uint64_t const timestamp) {
    auto const upper{data.lower_bound(timestamp)};
    if (upper == std::cbegin(data)) {
        return upper;
    } else if (upper == std::cend(data)) {
        return std::prev(upper);
    }

    auto const lower{std::prev(upper)};

    uint64_t const upper_delta{upper->first - timestamp};
    uint64_t const lower_delta{timestamp - lower->first};

    return lower_delta <= upper_delta ? lower : upper;
}

// TODO TEST? and split between hpp and cpp?
// NOTE(Jack): We need this kinda funky looking logic because we are dealing with unsigned types and need to worry about
// NOT creating negative numbers that will underflow. Probably was a dumb idea to use an unsigned type in the first
// place.
bool IsWithinThreshold(uint64_t const lhs, uint64_t const rhs, uint64_t const threshold_ns) {
    uint64_t const delta{lhs > rhs ? lhs - rhs : rhs - lhs};

    return delta <= threshold_ns;
}

// NOTE ALL CAMERAS GET SYNCED ONLY TO THE RIG FRAMES _ MEANS SOME FRAMES WILL HAVE ONE OR MORE OR NOT TARGETS
void BundleAdjustment::AddCamera(CameraProblemInput const& camera, uint64_t const max_sync_delta_ns,
                                 bool const optimize_intrinsic, bool const optimize_extrinsic, Problem& problem) {
    problem.cameras.emplace(
        camera.camera_id,
        Camera{camera.camera_info, {camera.intrinsic, camera.extrinsic}, {optimize_intrinsic, optimize_extrinsic}});

    // NOTE(Jack): Kinda surprisingly but this is the place where all the time synchronization happens!
    // TODO NOTE ON SYNC ALGO AND ITS LIMITS!
    TargetSamples remaining_targets{camera.targets};
    for (auto const& [frame_timestamp_ns, _] : problem.rig_poses) {
        auto const target_it{FindClosest(remaining_targets, frame_timestamp_ns)};
        if (target_it == std::cend(remaining_targets)) {
            continue;
        }

        auto const sample_timestamp_ns{target_it->first};
        if (not IsWithinThreshold(sample_timestamp_ns, frame_timestamp_ns, max_sync_delta_ns)) {
            continue;
        }

        problem.observations.push_back(
            {camera.camera_id, sample_timestamp_ns, frame_timestamp_ns, target_it->second.bundle});

        // Remove it so a double match cannot happen.
        remaining_targets.erase(target_it);
    }
}

std::vector<ReprojectionError> EvaluateResiduals(BundleAdjustment::Problem const& ba_problem) {
    std::vector<ReprojectionError> errors;
    errors.reserve(std::size(ba_problem.observations));

    for (auto const& [camera_id, sample_timestamp_ns, frame_timestamp_ns, bundle] : ba_problem.observations) {
        // cppcheck-suppress ignoredReturnValue
        auto const& [camera_info, camera_state, _]{ba_problem.cameras.at(camera_id)};
        if (not ba_problem.rig_poses.contains(frame_timestamp_ns)) {
            continue;
        }
        auto const& rig_pose{ba_problem.rig_poses.at(frame_timestamp_ns)};

        std::vector<double const*> parameter_blocks;
        parameter_blocks.push_back(camera_state.intrinsic.value.data());
        parameter_blocks.push_back(camera_state.extrinsic.data());
        parameter_blocks.push_back(rig_pose.value.data());

        auto const& [pixels, points]{bundle};

        // NOTE(Jack): Eigen is column major by default. Which means that if you just make a default array here and pass
        // the row pointer blindly into the EvaluateResidualBlock function it will not fill out the row but actually two
        // column elements! That is the reason why we have to specifically specify RowMajor here!
        Eigen::Array<double, Eigen::Dynamic, 2, Eigen::RowMajor> residuals_i{pixels.rows(), 2};
        for (Eigen::Index i{0}; i < pixels.rows(); ++i) {
            ceres::CostFunction const* const cost_function{
                cost_functions::Create(camera_info.camera_model, camera_info.bounds, pixels.row(i), points.row(i))};

            cost_function->Evaluate(parameter_blocks.data(), residuals_i.row(i).data(), nullptr);

            // TODO(Jack): Should we use a smart pointer instead?
            delete cost_function;
        }

        // NOTE(Jack): This is a critical point in the time synchronization logic! We use the original sample timestamps
        // here because these errors will get written into the database and we need to satisfy foreign key constraints.
        errors.push_back({camera_id, sample_timestamp_ns, residuals_i});
    }

    return errors;
}

}  // namespace  reprojection::optimization
