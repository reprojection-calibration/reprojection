#include "optimization/bundle_adjustment.hpp"

#include <ceres/loss_function.h>

#include <ranges>

#include "cost_functions/reprojection_error.hpp"
#include "time_synchronization/time_synchronization.hpp"

namespace reprojection::optimization {

// ERROR(Jack): What is a frame has too few valid pixels to actually constrain the pose? Should we entirely skip
// that frame? Or what if in general we have a minimum required of points per frame threshold?
std::pair<BundleAdjustment::Result, CeresState> BundleAdjustment::Solve(Problem const& ba_problem,
                                                                        int const num_threads) {
    // TODO(Jack): It is a little messy how we construct the result from just part of the problem, and then iterate over
    // the problem below but ignore the part that we copied to the result and use the result instead. Really not the end
    // of the world but I feel like I am missing the plotline.
    Result result{ba_problem};

    CeresState ceres_state{ceres::TAKE_OWNERSHIP, ceres::DENSE_SCHUR};
    ceres_state.solver_options.num_threads = num_threads;
    ceres::Problem ceres_problem{ceres_state.problem_options};

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

    ceres::Solve(ceres_state.solver_options, &ceres_problem, &ceres_state.solver_summary);

    return {result, ceres_state};
}

// TODO UNIT TEST!
// TODO UNIT TEST!
// TODO UNIT TEST!
BundleAdjustment::Problem BundleAdjustment::MultiCamProblem(CameraProblemInput const& cam0, Frames const& cam0_poses,
                                                            std::span<CameraProblemInput const> cams,
                                                            uint64_t const max_sync_delta_ns) {
    // TODO(Jack): Maybe one day we can engineer the need for this check away, but right now we do not guarantee that
    // the rig reference has optimize_extrinsic==false and an identity transform. So to solve our headaches prematurely
    // we add this check for both properties, but like I said hopefully one day we can engineer this away!
    if (cam0.optimize_extrinsic or cam0.extrinsic.sum() > 1e-12) {
        throw std::logic_error(
            "Cannot optimize the extrinsic of the rig defining reference camera or set it to anything besides "
            "identity!");
    }

    Problem problem{cam0.camera_id, cam0_poses};
    AddCamera(cam0, max_sync_delta_ns, problem);

    for (auto const& cam_i : cams) {
        AddCamera(cam_i, max_sync_delta_ns, problem);
    }

    return problem;
}  // LCOV_EXCL_LINE

// TODO(Jack): Refactor this to use the AddCamera method! We repeate the observation iteration which is not so nice.
BundleAdjustment::Problem BundleAdjustment::SingleCamProblem(CameraInfo const& camera_info, Intrinsic const& intrinsic,
                                                             TargetSamples const& targets, Frames const& frames,
                                                             bool const optimize_intrinsic, AssetId const camera_id) {
    CameraProblemInput const camera{
        camera_id, camera_info, intrinsic, targets, Array6d::Zero(), optimize_intrinsic, false,
    };

    return MultiCamProblem(camera, frames, {}, 0);
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

// NOTE(Jack): All observations get synced to the rig_poses. This means that there might be poses that only have one
// target (i.e. the reference camera's target) and there might be non-reference camera observations that do not sync and
// are therefore never used. This is a simplifying assumption and does not cost us much but prevents us from have to
// implement a more intricate "changing reference camera" problem construction logic. Maybe we are just missing the
// abstraction to do that simply?
void BundleAdjustment::AddCamera(CameraProblemInput const& camera, uint64_t const max_sync_delta_ns, Problem& problem) {
    problem.cameras.emplace(camera.camera_id, Camera{camera.camera_info,  // LCOV_EXCL_LINE
                                                     {camera.intrinsic, camera.extrinsic},
                                                     {camera.optimize_intrinsic, camera.optimize_extrinsic}});

    auto const timestamps{camera.targets | std::views::keys};
    std::set<uint64_t> remaining_targets{std::cbegin(timestamps), std::cend(timestamps)};

    for (auto const& [frame_timestamp_ns, _] : problem.rig_poses) {
        // TODO(Jack): Hand rolling the time synchronization logic here is not so nice, as we need it in multiple
        // places.
        auto const target_timestamps_it{time_synchronization::FindClosest(remaining_targets, frame_timestamp_ns)};
        if (target_timestamps_it == std::cend(remaining_targets)) {
            continue;  // LCOV_EXCL_LINE
        }

        auto const sample_timestamp_ns{*target_timestamps_it};
        if (not time_synchronization::IsWithinThreshold(sample_timestamp_ns, frame_timestamp_ns, max_sync_delta_ns)) {
            continue;  // LCOV_EXCL_LINE
        }

        problem.observations.push_back(
            {camera.camera_id, sample_timestamp_ns, frame_timestamp_ns, camera.targets.at(sample_timestamp_ns).bundle});

        // Remove it so a double match cannot happen.
        remaining_targets.erase(target_timestamps_it);
    }
}

transforms::RigState ToRigState(BundleAdjustment::Result const& result) {
    std::vector<Extrinsic> rig_cam_extrinsics;
    for (auto const& [camera_id, state_i] : result.camera_states) {
        // The Extrinsics() type does not allow self-connections/cycles!
        if (camera_id == result.rig_frame_asset_id) {
            continue;
        }

        // TODO(Jack): I do not know why this came as a little bit of a surprise to me that our extrinsic configuration
        // optimizes all the cameras with respect to the reference camera (see how rig_frame_asset_id is hardcoded
        // here). I think we need to think hard about this and make that is what we want! For some reason I thought it
        // would be that the extrinsic goes from neighbouring camera to neighbouring camera in a chain, but our current
        // setup is a tree. If we went from camera to camera I think this would make the optimization process ugly
        // because then we would need to chain them together and our cost function would have to accept a variable
        // number of extrinsics chained together! Does that make sense? We need to do some thinking here!!!
        Extrinsic const extrinsic_i{camera_id, result.rig_frame_asset_id, state_i.extrinsic};
        rig_cam_extrinsics.push_back(extrinsic_i);
    }

    return transforms::RigState{result.rig_frame_asset_id, result.rig_poses,
                                transforms::Extrinsics{{rig_cam_extrinsics}}};
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

        // NOTE(Jack): Here you see clearly how the observation was matched to the rig pose via the "frame" timestamp
        // (i.e. the approximate synchronization) but the output is saved out to the database under the original
        // "sample" timestamp so that the foreign key relationships are preserved.
        errors.push_back({camera_id, sample_timestamp_ns, residuals_i});
    }

    return errors;
}

}  // namespace  reprojection::optimization
